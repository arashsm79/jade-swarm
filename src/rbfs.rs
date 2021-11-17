use crate::problem::JadeSwarm;
use crate::problem::Node;

use std::collections::BinaryHeap;
use std::rc::Rc;

impl JadeSwarm {

/*
* --------------------------------------
* --------------------------------------
*  Recursive Best First Search
* --------------------------------------
* --------------------------------------
*/
    pub fn recursive_best_first_search(&self) -> Option<Vec<Node>> {
        let g = |_node: &Node| -> i32 { 1 };

        let initial_node = Node {
            state: self.initial_state.clone(),
            cost: 0,
            path_cost: 0,
            selected_node_id: 0,
            parent: None
        };

        let (solution, _fvalue) = self.rbfs(initial_node, i32::MAX, g);
        JadeSwarm::trace_back_path_with_parent(solution)
    }

    fn rbfs_h(&self, node: &Node) -> i32 {
            let x = f64::from(
                node.state.config.len() as i32
                    - node.state.config.iter().filter(|&b| (*b) == true).count() as i32,
            ) / self.maximum_branching_factor as f64;
            x.ceil() as i32
    }

    fn rbfs_expand(&self, node: &Node, g: fn(&Node) -> i32) -> Vec<Node>
    {
        let mut expanded_nodes = Vec::new();
        for i in 0..node.state.config.len() {
            if i != node.selected_node_id {
                let next_node = Node {
                    state: self.successor(&node.state, i),
                    cost: 0,
                    path_cost: 0,
                    selected_node_id: i,
                    parent: Some(Rc::new((*node).clone()))
                };
                let path_cost = node.path_cost + g(&next_node);
                expanded_nodes.push(Node {
                    path_cost,
                    cost: path_cost + self.rbfs_h(&next_node),
                    ..next_node
                })
                }
        }
        expanded_nodes
    }

    fn rbfs(&self, node: Node, f_limit: i32,  g: fn(&Node) -> i32) -> (Option<Node>, i32)
    {
        if JadeSwarm::is_goal(&node.state) {
            return (Some(node), f_limit);
        }

        let mut node_successors: BinaryHeap<Node> = BinaryHeap::new();

        for mut child in self.rbfs_expand(&node, g) {
            child.cost = std::cmp::max(child.cost, node.cost);
            node_successors.push(child);
        }

        loop {
            if let Some(mut best) = node_successors.pop() {
                if best.cost > f_limit {
                    return (None, best.cost)
                }
                if let Some(alternative) = node_successors.pop() {
                    let (result, best_f) = self.rbfs(best.clone(), std::cmp::min(f_limit, alternative.cost), g);
                    if result != None {
                        return (result, best_f)
                    }
                    best.cost = best_f;
                    node_successors.push(best);
                }
            }
        }
    }
}
