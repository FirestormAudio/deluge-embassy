use deluge_bsp::sd;
use log::{error, info};

/// SD card probe task.
///
/// Initialises the SD controller and reads the first sector (the partition
/// table / boot block).  Logs the first 16 bytes to RTT so you can verify
/// the card is being read correctly.
#[embassy_executor::task]
pub(crate) async fn sd_task() {
    info!("SD: initialising");
    match sd::init().await {
        Ok(()) => {
            info!("SD: card ready (HC={})", sd::is_hc());
            let mut buf = [0u8; 512];
            match sd::read_sector(0, &mut buf).await {
                Ok(()) => {
                    info!(
                        "SD: sector 0 first 16 bytes: {:02x} {:02x} {:02x} {:02x} \
                         {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} \
                         {:02x} {:02x} {:02x} {:02x}",
                        buf[0],
                        buf[1],
                        buf[2],
                        buf[3],
                        buf[4],
                        buf[5],
                        buf[6],
                        buf[7],
                        buf[8],
                        buf[9],
                        buf[10],
                        buf[11],
                        buf[12],
                        buf[13],
                        buf[14],
                        buf[15],
                    );
                }
                Err(e) => error!("SD: read_sector(0) failed: {:?}", e),
            }
        }
        Err(e) => error!("SD: init failed: {:?}", e),
    }
}
