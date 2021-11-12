use crate::problem::JadeSwarm;
use crate::problem::Node;
use crate::problem::State;

use std::collections::VecDeque;
use std::collections::HashMap;
/*
* --------------------------------------
* --------------------------------------
*  Breadth First Search
* --------------------------------------
* --------------------------------------
*/

impl JadeSwarm { 
    pub fn breadth_first_search(&self) -> Option<Vec<Node>> {
        // Do the search
        let mut reached: HashMap<State, Option<Node>> = HashMap::new();
        let mut frontier: VecDeque<Node> = VecDeque::new();
        let initial_node = Node {
            state: self.initial_state.clone(),
            cost: 0,
            path_cost: 0,
            selected_node_id: 0,
        };

        let mut goal_node: Option<Node> = None;
        if JadeSwarm::is_goal(&self.initial_state) {
            goal_node = Some(initial_node.clone())
        } else {
            reached.insert(initial_node.state.clone(), None);
            frontier.push_back(initial_node);
        }

        'outer: while let Some(node) = frontier.pop_front() {
            for child in self.uninformed_expand(&node) {
                if JadeSwarm::is_goal(&child.state) {
                    reached.insert(child.state.clone(), Some(node));
                    goal_node = Some(child);
                    break 'outer;
                }
                if !reached.contains_key(&child.state) {
                    reached.insert(child.state.clone(), Some(node.clone()));
                    frontier.push_back(child);
                }
            }
        }

        // Trace back path
        JadeSwarm::trace_back_path(goal_node, reached)
    }
}
