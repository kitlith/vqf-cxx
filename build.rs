fn main() {
    // std::env::set_var("AUTOCXX_RS", "generated/");
    // autocxx_build::Builder::new("src/lib.rs", &["vqf/vqf/cpp/"])
    //     .build()?
    //     .file("vqf/vqf/cpp/vqf.cpp")
    //     .compile("autocxx-vqf");

    // cxx_build::bridge("generated/autocxx-ffi-default-gen.rs")
    //     .file("vqf/vqf/cpp/vqf.cpp")
    //     .include("vqf/vqf/cpp/")
    //     .compile("cxx-vqf");

    cc::Build::new()
        .cpp(true)
        .cpp_link_stdlib(None)
        .include("vqf/vqf/cpp/")
        .file("vqf/vqf/cpp/vqf.cpp")
        .file("generated/autocxx-ffi-default-gen.rs.cc")
        .compile("cxx-vqf");

    println!("cargo:rerun-if-changed=generated/autocxx-ffi-default-gen.rs");
    println!("cargo:rerun-if-changed=generated/autocxxgen_ffi.h");
    println!("cargo:rerun-if-changed=generated/cxx.h");
    println!("cargo:rerun-if-changed=generated/autocxx-ffi-default-gen.rs.h");
    println!("cargo:rerun-if-changed=generated/autocxx-ffi-default-gen.rs.cc");
}
