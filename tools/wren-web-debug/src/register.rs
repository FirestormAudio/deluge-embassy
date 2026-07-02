//! Wire deluge's generic binding bodies into wren-core's foreign registries.
//!
//! The enumeration of *which* bindings exist lives in ONE place —
//! [`deluge_wren_core::register_foreign`] — so this crate never re-lists the
//! ~37 method signatures. We just adapt each generic body `fn(&CoreSlots)` into
//! a wren-core [`ForeignMethodFn::Closure`] that builds a [`CoreSlots`] for the
//! call.

use std::rc::Rc;

use wren_core::error::WrenResult;
use wren_core::foreign::{ForeignClassRegistry, ForeignMethodRegistry, WrenSlotApi};
use wren_core::objects::ForeignMethodFn;

use crate::slotapi_wrencore::CoreSlots;

/// Register every deluge foreign class + method into the given wren-core
/// registries.
pub fn register_all(mreg: &mut ForeignMethodRegistry, creg: &mut ForeignClassRegistry) {
    deluge_wren_core::register_foreign::<CoreSlots>(
        |module, class, is_static, sig, body| {
            let f = move |api: &dyn WrenSlotApi| -> WrenResult<()> {
                let cs = CoreSlots::new(api);
                body(&cs);
                Ok(())
            };
            mreg.register(
                module,
                class,
                is_static,
                sig,
                ForeignMethodFn::Closure(Rc::new(f)),
            );
        },
        |module, class, alloc| {
            let f = move |api: &dyn WrenSlotApi| -> WrenResult<()> {
                let cs = CoreSlots::new(api);
                alloc(&cs);
                Ok(())
            };
            creg.register(module, class, ForeignMethodFn::Closure(Rc::new(f)), None);
        },
    );
}
