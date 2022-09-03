fn main() -> miette::Result<()> {
    autocxx_build::Builder::new("src/lib.rs", &["vqf/vqf/cpp/"])
        .build()?
        .file("vqf/vqf/cpp/vqf.cpp")
        .compile("autocxx-vqf");

    println!("cargo:rerun-if-changed=src/lib.rs");

    Ok(())
}
