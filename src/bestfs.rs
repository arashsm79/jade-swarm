use crate::problem::JadeSwarm;
use crate::problem::Node;
use crate::problem::State;

use std::collections::{HashMap, BinaryHeap};

impl JadeSwarm {
/*
* --------------------------------------
* --------------------------------------
*  Best First Search
* --------------------------------------
* --------------------------------------
*/
    pub fn best_first_search<F>(&self, g: fn(&Node) -> i32, h: F) -> Option<Vec<Node>>
    where
        F: Fn(&Node) -> i32,
    {
        let mut reached: HashMap<State, Option<Node>> = HashMap::new();
        let mut frontier: BinaryHeap<Node> = BinaryHeap::new();
        let initial_node = Node {
            state: self.initial_state.clone(),
            cost: 0,
            path_cost: 0,
            selected_node_id: 0,
        };
        let mut goal_node: Option<Node> = None;

        frontier.push(initial_node.clone());
        reached.insert(self.initial_state.clone(), None);

        while let Some(node) = frontier.pop() {
            if JadeSwarm::is_goal(&node.state) {
                goal_node = Some(node);
                break;
            }

            for child in self.informed_expand(&node, g, &h) {
                // TODO make below condition more readable
                if !reached.contains_key(&child.state)
                    || child.cost
                        < if let Some(node) = reached
                            .get(&child.state)
                            .unwrap()
                            .as_ref() {
                            node.cost
                        } else {
                            0
                        }
                {
                    reached.insert(child.state.clone(), Some(node.clone()));
                    frontier.push(child);
                }
            }
        }

        // Trace back path
        JadeSwarm::trace_back_path(goal_node, reached)
    }
}
