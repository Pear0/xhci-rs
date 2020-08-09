#[derive(Default, Debug, Copy, Clone)]
pub struct XHCIQuirks {
    pub no_reset_before_address_device: bool,
}