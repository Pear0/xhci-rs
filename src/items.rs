use alloc::boxed::Box;

#[derive(Clone, Debug, Default)]
pub struct Port {
    pub port_id: u8,
    pub slot_id: u8,
    pub parent: Option<Box<Port>>,
    pub is_low_or_full_speed: bool,
}

impl Port {
    pub fn new_from_root(port_id: u8) -> Self {
        Port {
            port_id,
            slot_id: 0,
            parent: None,
            is_low_or_full_speed: false,
        }
    }

    pub fn child_port(&self, port_id: u8) -> Self {
        Port {
            port_id,
            slot_id: 0,
            parent: Some(Box::new(self.clone())),
            is_low_or_full_speed: false,
        }
    }

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




