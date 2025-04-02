use std::time::{Duration, Instant};

use rusb::{UsbContext, constants::LIBUSB_REQUEST_GET_DESCRIPTOR};
#[cfg(feature = "dma")]
use rusb_async::DeviceHandleExt;
use rusb_async::{CancellationToken, ControlPacket, LibusbTransfer2};
#[cfg(feature = "zerocopy")]
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
    #[cfg(feature = "dma")]
    let mut buf = unsafe { handle.new_usb_mem(0x16000).unwrap() };
    #[cfg(not(feature = "dma"))]
    let mut buf = rusb_async::UsbMemMut::with_capacity(0x16000);
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
    #[cfg(feature = "zerocopy")]
    pkt.write_to_prefix(&mut buf[..]).unwrap();
    #[cfg(not(feature = "zerocopy"))]
    unsafe {
        buf[..size_of::<ControlPacket>()]
            .as_mut_ptr()
            .cast::<ControlPacket>()
            .write(pkt)
    };

    let mut transfer =
        unsafe { LibusbTransfer2::new_with_zero_packets().into_ctrl(&handle, buf, Duration::from_millis(600)) };

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
    let mut cancel_transfer2 = cancel_transfer.clone();
    rt.block_on(async move {
        println!("about to run transfer");
        let result = transfer.submit_and_wait(&mut cancel_transfer2).await;
        println!("{result:?}");
        if result.is_ok() {
            println!("{:?}", transfer.into_parts().unwrap().1);
        }
    });

    println!("stopping event handler");
    event_handler.cancel();
    drop(handle);
    println!("event handler output: {:?}", thread.join());
}
