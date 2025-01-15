use std::time::Duration;

use bytes::{BufMut, BytesMut};
use rusb::UsbContext;
use rusb_async::{ControlPacket, Transfer};
use tokio_util::sync::CancellationToken;
use zerocopy::IntoBytes;

fn main() {
    let ctx = rusb::Context::new().unwrap();
    let handle = ctx.open_device_with_vid_pid(0x0c45, 0x7016).unwrap();

    handle.set_auto_detach_kernel_driver(true).unwrap();
    for interface in 0..16 {
        if let Ok(true) = handle.kernel_driver_active(interface) {
            handle.detach_kernel_driver(interface).unwrap();
        }
    }
    handle.unconfigure().unwrap();
    let ctx = handle.context().clone();
    handle.reset().unwrap();

    let mut buf = BytesMut::with_capacity(size_of::<ControlPacket>() + 18);
    buf.put_bytes(0, size_of::<ControlPacket>());
    let pkt = ControlPacket {
        bm_request_type: rusb::request_type(
            rusb::Direction::In,
            rusb::RequestType::Standard,
            rusb::Recipient::Device,
        ),
        b_request: 0x06,
        w_value: 0x01,
        w_index: 0,
        w_length: 18,
    };
    pkt.write_to_prefix(&mut buf[..]).unwrap();

    let mut transfer = unsafe { Transfer::new_ctrl(&handle, buf, Duration::from_millis(600)) };

    let cancel_token = CancellationToken::new();
    let event_handler = cancel_token.clone();
    let thread = std::thread::spawn(move || {
        println!("starting event loop");
        while !cancel_token.is_cancelled() {
            // println!("checking for events");
            ctx.handle_events(Some(Duration::from_secs(1))).unwrap();
        }
    });

    let rt = tokio::runtime::Runtime::new().unwrap();

    let cancel_transfer = CancellationToken::new();
    let cancel_transfer2 = cancel_transfer.clone();
    rt.block_on(async move {
        println!("about to run transfer");
        let result = transfer.submit(cancel_transfer2).await;
        println!("{result:?}");
    });

    println!("stopping event handler");
    event_handler.cancel();
    drop(handle);
    println!("event handler output: {:?}", thread.join());
}
