use std::time::{Duration, Instant};

use rusb::{constants::LIBUSB_REQUEST_GET_DESCRIPTOR, UsbContext};
use rusb_async::{ControlPacket, DeviceHandleExt, InnerTransfer};
use tokio_util::sync::CancellationToken;
use zerocopy::IntoBytes;

fn main() {
    let ctx = rusb::Context::new().unwrap();
    let handle = ctx
        .devices()
        .unwrap()
        .iter()
        .find(|dev| dev.bus_number() == 3 && dev.address() == 36)
        .unwrap()
        .open()
        .unwrap();

    handle.set_auto_detach_kernel_driver(true).unwrap();
    for interface in 0..16 {
        if let Ok(true) = handle.kernel_driver_active(interface) {
            handle.detach_kernel_driver(interface).unwrap();
        }
    }
    handle.unconfigure().unwrap();
    let ctx = handle.context().clone();
    handle.reset().unwrap();

    let needed = size_of::<ControlPacket>() + 18;
    let now = Instant::now();
    let mut buf = unsafe { handle.new_usb_mem(0x16000).unwrap() };
    let elapsed = now.elapsed();
    println!("took {elapsed:?} to map memory");
    unsafe { buf.set_len(needed) };
    let mut buf = buf.split_to(needed);
    let pkt = ControlPacket {
        bm_request_type: rusb::request_type(
            rusb::Direction::In,
            rusb::RequestType::Standard,
            rusb::Recipient::Device,
        ),
        b_request: LIBUSB_REQUEST_GET_DESCRIPTOR,
        w_value: 1 << 8,
        w_index: 0,
        w_length: 18,
    };
    pkt.write_to_prefix(&mut buf[..]).unwrap();

    let mut transfer =
        unsafe { InnerTransfer::new(0).into_ctrl(&handle, buf, Duration::from_millis(600)) };

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
        // SAFETY: We run this transfer to its completion.
        let result = unsafe { transfer.submit(cancel_transfer2) }.await;
        println!("{result:?}");
        if result.is_ok() {
            println!("{:?}", transfer.into_buf().unwrap());
        }
    });

    println!("stopping event handler");
    event_handler.cancel();
    drop(handle);
    println!("event handler output: {:?}", thread.join());
}
