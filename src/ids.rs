use crate::problem::JadeSwarm;
use crate::problem::Node;
use crate::problem::State;

use std::collections::VecDeque;
use std::collections::HashMap;
impl JadeSwarm {
/*
* --------------------------------------
* --------------------------------------
*  Iterative Deepening Search
* --------------------------------------
* --------------------------------------
*/
    fn depth_limited(&self, depth: usize) -> Option<Vec<Node>> {
        // Do the search
        let mut reached: HashMap<State, Option<Node>> = HashMap::new();
        let mut frontier: VecDeque<(Node, usize)> = VecDeque::new();
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
            frontier.push_back((initial_node.clone(), 0));
            reached.insert(initial_node.state, None);

            while let Some((node, l)) = frontier.pop_back() {
                if JadeSwarm::is_goal(&node.state) {
                    goal_node = Some(node);
                    break;
                }

                if l < depth {
                    for child in self.uninformed_expand(&node) {
                        if !reached.contains_key(&child.state) {
                            reached.insert(child.state.clone(), Some(node.clone()));
                            frontier.push_back((child, l + 1));
                        }
                    }
                }
            }
        }

        // Trace back path
        JadeSwarm::trace_back_path(goal_node, reached)
    }

    pub fn iterative_deepening_search(&self) -> Option<Vec<Node>> {
        for depth in 0..usize::MAX {
            println!("depth {}", depth);
            match self.depth_limited(depth) {
                Some(path) => return Some(path),
                None => continue,
            }
        }
        return None;
    }    
}
