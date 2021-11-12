use crate::problem::JadeSwarm;
use crate::problem::Node;
use crate::problem::State;

use std::collections::VecDeque;
use std::collections::HashMap;

impl JadeSwarm {
/*
* --------------------------------------
* --------------------------------------
*  Depth First Search
* --------------------------------------
* --------------------------------------
*/
    pub fn depth_first_search(&self) -> Option<Vec<Node>> {
        let mut reached: HashMap<State, Option<Node>> = HashMap::new();
        let mut frontier: VecDeque<(Node, Option<Node>)> = VecDeque::new();
        let initial_node = Node {
            state: self.initial_state.clone(),
            cost: 0,
            path_cost: 0,
            selected_node_id: 0,
        };

        let mut goal_node: Option<Node> = None;
        if JadeSwarm::is_goal(&self.initial_state) {
            goal_node = Some(initial_node)
        } else {
            frontier.push_back((initial_node.clone(), None));

            while let Some((node, previous_node)) = frontier.pop_back() {
                reached.insert(node.state.clone(), previous_node);
                if JadeSwarm::is_goal(&node.state) {
                    goal_node = Some(node);
                    break;
                }
                for child in self.uninformed_expand(&node) {
                    if !reached.contains_key(&child.state) {
                        frontier.push_back((child, Some(node.clone())));
                    }
                }
            }
        }

        // Trace back path
        JadeSwarm::trace_back_path(goal_node, reached)
    } 
}
