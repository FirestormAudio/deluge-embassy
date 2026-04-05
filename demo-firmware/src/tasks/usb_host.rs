use embassy_usb_driver::host::{DeviceEvent, UsbHostDriver};
use embassy_usb_host::UsbHost;
use log::{error, info};

use deluge_bsp::usb::Rusb1HostDriver;

/// Runs the USB host stack: waits for a device, enumerates it, then waits for
/// disconnect before looping.
///
/// Spawn this task *instead of* `usb_task` when the port is in host mode.
/// The `driver` must have been initialised via [`deluge_bsp::usb::init_host_mode`]
/// and the ISR wired to [`deluge_bsp::usb::hcd_int_handler`].
#[embassy_executor::task]
pub(crate) async fn usb_host_task(driver: Rusb1HostDriver) {
    info!("usb_host_task: running");
    let mut usb_host = UsbHost::new(driver);
    let mut config_buf = [0u8; 512];

    loop {
        let speed = usb_host.wait_for_connection().await;
        info!("USB host: device connected ({:?})", speed);

        match usb_host.enumerate(speed, &mut config_buf).await {
            Ok((dev_desc, addr, _)) => {
                info!(
                    "USB host: VID={:04x} PID={:04x} addr={}",
                    dev_desc.vendor_id, dev_desc.product_id, addr
                );
                // Wait for disconnect before accepting a new device.
                loop {
                    match usb_host.driver_mut().wait_for_device_event().await {
                        DeviceEvent::Disconnected => {
                            info!("USB host: device disconnected");
                            usb_host.free_address(addr);
                            break;
                        }
                        DeviceEvent::Connected(_) => {
                            // Spurious re-connect while already connected; ignore.
                        }
                    }
                }
            }
            Err(e) => {
                error!("USB host: enumeration failed: {:?}", e);
            }
        }
    }
}
