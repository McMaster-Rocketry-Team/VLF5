use core::cell::RefCell;

use embassy_sync::blocking_mutex::{Mutex, raw::RawMutex};
use embassy_time::Instant;
use firmware_common_new::{
    can_bus::{id::CanBusExtendedId, messages::node_status::NodeStatusMessage},
    vlp::packets::self_test_result::NodeStatus as VlpNodeStatus,
};
use heapless::{FnvIndexMap, Vec};

#[derive(defmt::Format, Debug, Clone)]
pub struct CanNode {
    pub typ: u8,
    pub id: u16,
    pub status: NodeStatusMessage,
    /// time since boot
    pub last_received_time_us: u64,
}

impl CanNode {
    pub fn is_online(&self) -> bool {
        Instant::now().as_micros() - self.last_received_time_us < 5_000_000
    }

    pub fn rebooted_in_last_5s(&self) -> bool {
        self.status.uptime_s < 5
    }
}

impl Into<VlpNodeStatus> for CanNode {
    fn into(self) -> VlpNodeStatus {
        if self.is_online() {
            VlpNodeStatus {
                health: self.status.health,
                mode: self.status.mode,
                rebooted_in_last_5s: self.rebooted_in_last_5s(),
                custom_status: self.status.custom_status_raw,
            }
        } else {
            VlpNodeStatus::offline()
        }
    }
}

pub struct CanCentral<R: RawMutex> {
    /// map node id to CanNode
    nodes: Mutex<R, RefCell<FnvIndexMap<u16, CanNode, 16>>>,
}

impl<R: RawMutex> CanCentral<R> {
    pub fn new() -> Self {
        Self {
            nodes: Mutex::new(RefCell::new(FnvIndexMap::new())),
        }
    }

    pub fn on_receive_node_status(
        &self,
        received_time_us: u64,
        id: &CanBusExtendedId,
        message: &NodeStatusMessage,
    ) {
        self.nodes.lock(|r| {
            let mut nodes = r.borrow_mut();
            if let Some(node) = nodes.get_mut(&id.node_id) {
                node.status = message.clone();
                node.last_received_time_us = received_time_us;
            } else {
                if nodes.len() >= nodes.capacity() {
                    let least_recent_node_id = nodes
                        .values()
                        .min_by_key(|node| node.last_received_time_us)
                        .unwrap()
                        .id;
                    nodes.remove(&least_recent_node_id);
                }
                nodes
                    .insert(
                        id.node_id,
                        CanNode {
                            typ: id.node_type,
                            id: id.node_id,
                            status: message.clone(),
                            last_received_time_us: received_time_us,
                        },
                    )
                    .ok();
            }
        });
    }

    pub fn get_nodes<const N: usize>(&self, node_type: u8) -> Vec<CanNode, N> {
        self.nodes.lock(|r| {
            let nodes = r.borrow();
            let mut vec = Vec::new();
            vec.extend(
                nodes
                    .values()
                    .filter(move |node| node.typ == node_type)
                    .take(N)
                    .cloned(),
            );

            vec
        })
    }

    pub fn get_all_nodes(&self) -> Vec<CanNode, 16> {
        self.nodes.lock(|r| {
            let nodes = r.borrow();
            let mut vec = Vec::new();
            vec.extend(nodes.values().cloned());

            vec
        })
    }

    pub fn clear(&self) {
        self.nodes.lock(|r| {
            let mut nodes = r.borrow_mut();

            nodes.clear();
        })
    }
}
