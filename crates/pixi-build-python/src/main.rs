// Binary entry point - all implementation is in lib.rs

#[tokio::main]
async fn main() {
    pixi_build_python::run().await
}
