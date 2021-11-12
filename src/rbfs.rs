use crate::problem::JadeSwarm;
use crate::problem::Node;
use crate::problem::RBFSNode;

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
        let g = |_node: &RBFSNode| -> i32 { 1 };

        let initial_node = RBFSNode {
            state: self.initial_state.clone(),
            cost: 0,
            path_cost: 0,
            selected_node_id: 0,
            parent: None
        };

        let (solution, _fvalue) = self.rbfs(initial_node, i32::MAX, g);
        JadeSwarm::rbfs_trace_back_path(solution)
    }

    fn rbfs_h(&self, node: &RBFSNode) -> i32 {
            let x = f64::from(
                node.state.config.len() as i32
                    - node.state.config.iter().filter(|&b| (*b) == true).count() as i32,
            ) / self.maximum_branching_factor as f64;
            x.ceil() as i32
    }

    fn rbfs_expand(&self, node: &RBFSNode, g: fn(&RBFSNode) -> i32) -> Vec<RBFSNode>
    {
        let mut expanded_nodes = Vec::new();
        for i in 0..node.state.config.len() {
            let next_node = RBFSNode {
                state: self.successor(&node.state, i),
                cost: 0,
                path_cost: 0,
                selected_node_id: i,
                parent: Some(Rc::new((*node).clone()))
            };
            let path_cost = node.path_cost + g(&next_node);
            expanded_nodes.push(RBFSNode {
                path_cost,
                cost: path_cost + self.rbfs_h(&next_node),
                ..next_node
            })
        }
        expanded_nodes
    }

    fn rbfs(&self, node: RBFSNode, f_limit: i32,  g: fn(&RBFSNode) -> i32) -> (Option<RBFSNode>, i32)
    {
        if JadeSwarm::is_goal(&node.state) {
            return (Some(node), f_limit);
        }

        let mut node_successors: BinaryHeap<RBFSNode> = BinaryHeap::new();

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

    fn rbfs_trace_back_path(
        goal_node: Option<RBFSNode>,
    ) -> Option<Vec<Node>> {
        match goal_node {
            Some(goal_node_unwrapped) => {
                // The goal state was found
                let mut path: Vec<Node> = Vec::new();
                path.push(Node {
                    state: goal_node_unwrapped.state,
                    selected_node_id: goal_node_unwrapped.selected_node_id,
                    cost: goal_node_unwrapped.cost,
                    path_cost: goal_node_unwrapped.path_cost
                });
                let mut p = goal_node_unwrapped.parent;
                loop {
                    match p {
                        Some(parent) => {
                            // Node has a parent
                            path.push(Node {
                                state: parent.state.clone(),
                                selected_node_id: parent.selected_node_id,
                                cost: parent.cost,
                                path_cost: parent.path_cost
                            });
                            p = parent.parent.clone();
                        }
                        None => {
                            // Node doesn't a have a parent (we have reached the initial state)
                            break;
                        }
                    }
                }
                return Some(path);
            }
            None => {
                // No solutions were found
                return None;
            }
        }
    }

}
