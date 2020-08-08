use alloc::boxed::Box;

#[derive(Clone, Debug, Default)]
pub struct Port {
    pub port_id: u8,
    pub slot_id: u8,
    pub parent: Option<Box<Port>>,
}

impl Port {
    pub fn get_root_port_id(&self) -> u8 {
        match &self.parent {
            None => self.port_id,
            Some(parent) => parent.get_root_port_id(),
        }
    }

    pub fn construct_route_string(&self) -> u32 {
        let mut string = 0u32;
        let mut me = self;
        loop {
            match &me.parent {
                Some(p) => {
                    string <<= 4;
                    string |= me.port_id as u32;
                    me = p.as_ref();
                }
                None => return string,
            }
        }
    }
}




