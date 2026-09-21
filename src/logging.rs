use std::fs::{File, OpenOptions};
use std::io::Write;
use std::path::PathBuf;
use std::sync::Mutex;

pub const LOG_FILE_NAME: &str = "cesium.log";

/// A writer that outputs to both stderr and a shared file, flushing immediately.
struct DualWriter {
    file: Mutex<File>,
}

impl Write for DualWriter {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        let _ = std::io::stderr().write_all(buf);
        if let Ok(mut f) = self.file.lock() {
            let _ = f.write_all(buf);
            let _ = f.flush();
        }
        Ok(buf.len())
    }

    fn flush(&mut self) -> std::io::Result<()> {
        let _ = std::io::stderr().flush();
        if let Ok(mut f) = self.file.lock() {
            let _ = f.flush();
        }
        Ok(())
    }
}

/// Initializes file and console logging to `cesium.log` and installs a fatal panic hook.
pub fn init_logging() -> PathBuf {
    let log_path = PathBuf::from(LOG_FILE_NAME);

    // Create / truncate log file for a clean session log
    let file = OpenOptions::new()
        .create(true)
        .write(true)
        .truncate(true)
        .open(&log_path)
        .expect("Failed to create cesium.log");

    // Startup banner
    let args: Vec<String> = std::env::args().collect();
    let current_dir = std::env::current_dir()
        .map(|p| p.display().to_string())
        .unwrap_or_else(|_| "unknown".to_string());

    let banner = format!(
        "================================================================================\n\
         CesiumRS Session Log - Started at {}\n\
         Target OS:        {} ({})\n\
         Process ID:       {}\n\
         Working Directory: {}\n\
         Command Line:     {}\n\
         ================================================================================\n",
        humantime_now(),
        std::env::consts::OS,
        std::env::consts::ARCH,
        std::process::id(),
        current_dir,
        args.join(" ")
    );

    let _ = (&file).write_all(banner.as_bytes());
    let _ = (&file).flush();

    // 1. Install Fatal Panic Hook with Forced Backtrace
    let panic_log_path = log_path.clone();
    let default_hook = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        let thread = std::thread::current();
        let thread_name = thread.name().unwrap_or("unnamed");

        let payload = if let Some(s) = info.payload().downcast_ref::<&str>() {
            *s
        } else if let Some(s) = info.payload().downcast_ref::<String>() {
            s.as_str()
        } else {
            "Box<dyn Any>"
        };

        let location = info
            .location()
            .map(|l| format!("{}:{}:{}", l.file(), l.line(), l.column()))
            .unwrap_or_else(|| "unknown location".to_string());

        let backtrace = std::backtrace::Backtrace::force_capture();

        let panic_report = format!(
            "\n\
             !!!!!!!!!!!!!!!!!!!!!!!! FATAL APPLICATION CRASH !!!!!!!!!!!!!!!!!!!!!!!!\n\
             Thread:    '{}'\n\
             Location:  {}\n\
             Reason:    {}\n\
             Time:      {}\n\
             -------------------------------- BACKTRACE --------------------------------\n\
             {}\n\
             !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n",
            thread_name,
            location,
            payload,
            humantime_now(),
            backtrace
        );

        eprintln!("{}", panic_report);

        if let Ok(mut f) = OpenOptions::new().append(true).open(&panic_log_path) {
            let _ = f.write_all(panic_report.as_bytes());
            let _ = f.flush();
        }

        default_hook(info);
    }));

    // 2. Install Fatal Native Signal Handlers (SIGSEGV, SIGABRT, etc.)
    #[cfg(unix)]
    unsafe {
        install_signal_handlers();
    }

    // 3. Configure env_logger to write to DualWriter (stderr + cesium.log)
    let dual_writer = DualWriter {
        file: Mutex::new(file),
    };

    let mut builder = env_logger::Builder::from_default_env();
    builder
        .filter_level(log::LevelFilter::Debug)
        .filter_module("wgpu", log::LevelFilter::Warn)
        .filter_module("wgpu_core", log::LevelFilter::Warn)
        .filter_module("wgpu_hal", log::LevelFilter::Warn)
        .filter_module("naga", log::LevelFilter::Warn)
        .filter_module("reqwest", log::LevelFilter::Warn)
        .filter_module("hyper", log::LevelFilter::Warn)
        .filter_module("tokio", log::LevelFilter::Warn)
        .filter_module("cesium_rs", log::LevelFilter::Debug)
        .filter_module("cesium_engine", log::LevelFilter::Debug)
        .filter_module("cesium_flight", log::LevelFilter::Debug)
        .format_timestamp_millis()
        .target(env_logger::Target::Pipe(Box::new(dual_writer)));

    let _ = builder.try_init();

    log::info!("Logger initialized. Logging to stderr and {}", log_path.display());
    log_path
}

#[cfg(unix)]
unsafe fn install_signal_handlers() {
    use std::sync::atomic::{AtomicBool, Ordering};
    static HANDLER_INSTALLED: AtomicBool = AtomicBool::new(false);
    if HANDLER_INSTALLED.swap(true, Ordering::SeqCst) {
        return;
    }

    extern "C" fn handle_signal(sig: libc::c_int) {
        let sig_name = match sig {
            libc::SIGSEGV => "SIGSEGV (Segmentation Fault)",
            libc::SIGABRT => "SIGABRT (Abort / Driver Assertion)",
            libc::SIGBUS => "SIGBUS (Bus Error)",
            libc::SIGILL => "SIGILL (Illegal Instruction)",
            libc::SIGFPE => "SIGFPE (Floating Point Exception)",
            libc::SIGTERM => "SIGTERM (Termination Signal)",
            _ => "Unknown Signal",
        };

        let backtrace = std::backtrace::Backtrace::force_capture();
        let crash_report = format!(
            "\n\
             !!!!!!!!!!!!!!!!!!!!!!!! FATAL NATIVE SIGNAL CRASH !!!!!!!!!!!!!!!!!!!!!!!!\n\
             Signal:    {} ({})\n\
             Time:      {}\n\
             -------------------------------- BACKTRACE --------------------------------\n\
             {}\n\
             !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n",
            sig_name,
            sig,
            humantime_now(),
            backtrace
        );

        eprintln!("{}", crash_report);

        if let Ok(mut f) = OpenOptions::new().append(true).open(LOG_FILE_NAME) {
            let _ = f.write_all(crash_report.as_bytes());
            let _ = f.flush();
        }

        unsafe {
            libc::signal(sig, libc::SIG_DFL);
            libc::raise(sig);
        }
    }

    let signals = [
        libc::SIGSEGV,
        libc::SIGABRT,
        libc::SIGBUS,
        libc::SIGILL,
        libc::SIGFPE,
        libc::SIGTERM,
    ];

    for &sig in &signals {
        libc::signal(sig, handle_signal as *const () as libc::sighandler_t);
    }
}

fn humantime_now() -> String {
    match std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH) {
        Ok(d) => {
            let secs = d.as_secs();
            let millis = d.subsec_millis();
            format!("{}.{:03}s UNIX", secs, millis)
        }
        Err(_) => "unknown".to_string(),
    }
}
