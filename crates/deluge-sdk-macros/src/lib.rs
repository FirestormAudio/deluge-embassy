//! Procedural macros for the Deluge SDK.
//!
//! The only macro today is [`macro@app`], which turns a plain `async fn main`
//! into a complete firmware entry point — absorbing the platform bring-up
//! (heaps, clocks, interrupts, executor) and the panic handler that an app
//! author would otherwise hand-write.

use proc_macro::TokenStream;
use quote::quote;
use syn::{parse_macro_input, FnArg, ItemFn};

/// Mark an `async fn main` as the entry point of a Deluge app.
///
/// ```ignore
/// #![no_std]
/// #![no_main]
/// #![feature(impl_trait_in_assoc_type)]
/// use deluge::prelude::*;
///
/// #[deluge::app]
/// async fn main(dlg: Deluge) {
///     // your async code; the platform is already initialised.
/// }
/// ```
///
/// The annotated function must be `async`. It may take a single argument of
/// type [`Deluge`](../deluge/struct.Deluge.html); if omitted, the handle is
/// simply not bound.
///
/// ## Optional `setup`
/// Pass `#[deluge::app(setup = path::to::fn)]` to run a synchronous function
/// *after* clocks are up but *before* interrupts are enabled — for peripheral or
/// GIC bring-up that must happen with IRQs masked:
/// ```ignore
/// #[deluge::app(setup = setup)]
/// async fn main(dlg: Deluge) { /* interrupts on, executor running */ }
///
/// fn setup() { /* interrupts masked; register ISRs, configure GIC sources */ }
/// ```
///
/// ## What it expands to
/// - an Embassy task wrapping the function body,
/// - `extern "C" fn main` that runs `deluge::__rt::run(setup, spawn)` (logging →
///   heaps + clocks → `setup` → enable interrupts → executor), and
/// - a `#[panic_handler]`.
#[proc_macro_attribute]
pub fn app(args: TokenStream, item: TokenStream) -> TokenStream {
    // Parse optional `setup = path` from the attribute arguments.
    let mut setup_path: Option<syn::Path> = None;
    let arg_parser = syn::meta::parser(|meta| {
        if meta.path.is_ident("setup") {
            setup_path = Some(meta.value()?.parse()?);
            Ok(())
        } else {
            Err(meta.error("unknown #[deluge::app] argument (expected `setup = <path>`)"))
        }
    });
    parse_macro_input!(args with arg_parser);

    let setup_call = match setup_path {
        Some(path) => quote! { #path() },
        None => quote! {},
    };

    let func = parse_macro_input!(item as ItemFn);

    if func.sig.asyncness.is_none() {
        return syn::Error::new_spanned(
            &func.sig,
            "#[deluge::app] must be applied to an `async fn`",
        )
        .to_compile_error()
        .into();
    }
    if func.sig.inputs.len() > 1 {
        return syn::Error::new_spanned(
            &func.sig.inputs,
            "#[deluge::app] entry point takes at most one argument (the `Deluge` handle)",
        )
        .to_compile_error()
        .into();
    }

    let body = &func.block;
    let attrs = &func.attrs;

    // Bind the `Deluge` handle. Reuse the author's pattern and declared type
    // verbatim (e.g. `mut dlg: Deluge`) so `mut` bindings work and the type name
    // — typically imported via `deluge::prelude::*` — is genuinely referenced.
    // The value is always constructed through the canonical path so the binding
    // works regardless of how the author named the type.
    let bind_handle = match func.sig.inputs.first() {
        Some(FnArg::Typed(arg)) => {
            let pat = &arg.pat;
            let ty = &arg.ty;
            quote! { let #pat: #ty = ::deluge::Deluge::__new(spawner); }
        }
        Some(FnArg::Receiver(recv)) => {
            return syn::Error::new_spanned(recv, "#[deluge::app] entry point cannot take `self`")
                .to_compile_error()
                .into();
        }
        None => quote! { let _dlg: ::deluge::Deluge = ::deluge::Deluge::__new(spawner); },
    };

    quote! {
        #(#attrs)*
        // The SDK re-exports the embassy task attribute so the app doesn't need a
        // direct embassy-executor dependency (and so the right — device or host —
        // executor is selected automatically by target).
        #[::deluge::__rt::task(embassy_executor = ::deluge::__rt::embassy_executor)]
        async fn __deluge_app_main(spawner: ::deluge::__rt::Spawner) {
            #bind_handle
            #body
        }

        // Device entry point: a C `main` plus the panic handler the bare-metal
        // runtime needs.
        #[cfg(target_os = "none")]
        #[unsafe(no_mangle)]
        pub extern "C" fn main() -> ! {
            ::deluge::__rt::run(
                // Synchronous, interrupts-masked setup (empty unless `setup = …`).
                || { #setup_call; },
                |spawner: ::deluge::__rt::Spawner| {
                    // In this Embassy version `#[task]` returns a Result; the only
                    // failure is pool exhaustion, impossible for a single spawn here.
                    spawner.spawn(__deluge_app_main(spawner).unwrap());
                },
            )
        }

        #[cfg(target_os = "none")]
        #[panic_handler]
        fn __deluge_panic(info: &::core::panic::PanicInfo) -> ! {
            ::deluge::__rt::panic(info)
        }

        // Host entry point (the desktop simulator, via `cargo deluge sim`): a normal
        // `fn main` that hands off to the host runtime (which owns the GUI).
        #[cfg(all(not(target_os = "none"), feature = "sim"))]
        fn main() {
            ::deluge::__rt::host::run(
                || { #setup_call; },
                |spawner: ::deluge::__rt::Spawner| {
                    spawner.spawn(__deluge_app_main(spawner).unwrap());
                },
            )
        }

        // Native Linux entry point (`cargo deluge linux`): a normal `fn main`
        // that hands off to the linux runtime (which owns libdeluge).
        #[cfg(all(not(target_os = "none"), feature = "linux"))]
        fn main() {
            ::deluge::__rt::linux::run(
                || { #setup_call; },
                |spawner: ::deluge::__rt::Spawner| {
                    spawner.spawn(__deluge_app_main(spawner).unwrap());
                },
            )
        }
    }
    .into()
}
