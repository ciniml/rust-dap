fn main() {
    #[cfg(feature = "defmt")]
    {
        println!("cargo:rustc-link-arg=-Tdefmt.x");
    }
}
