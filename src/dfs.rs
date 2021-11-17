use crate::problem::JadeSwarm;
use crate::problem::Node;
use crate::problem::State;

use std::collections::VecDeque;
use std::collections::HashSet;

impl JadeSwarm {
/*
* --------------------------------------
* --------------------------------------
*  Depth First Search
* --------------------------------------
* --------------------------------------
*/
    pub fn depth_first_search_with_reached(&self) -> Option<Vec<Node>> {
        let mut reached: HashSet<State> = HashSet::new();
        let mut frontier: VecDeque<Node> = VecDeque::new();
        let initial_node = Node {
            state: self.initial_state.clone(),
            cost: 0,
            path_cost: 0,
            parent: None,
            selected_node_id: 0,
        };

        let mut goal_node: Option<Node> = None;
        if JadeSwarm::is_goal(&self.initial_state) {
            goal_node = Some(initial_node)
        } else {
            frontier.push_back(initial_node.clone());

            while let Some(node) = frontier.pop_back() {
                reached.insert(node.state.clone());
                if JadeSwarm::is_goal(&node.state) {
                    goal_node = Some(node);
                    break;
                }
                for child in self.uninformed_expand(&node) {
                    if !reached.contains(&child.state) {
                        frontier.push_back(child);
                    }
                }
            }
        }

        // Trace back path
        JadeSwarm::trace_back_path_with_parent(goal_node)
    } 

/// ---------------------------

    pub fn depth_first_search_without_cycle_checking(&self) -> Option<Vec<Node>> {
        let mut frontier: VecDeque<Node> = VecDeque::new();
        let initial_node = Node {
            state: self.initial_state.clone(),
            cost: 0,
            path_cost: 0,
            parent: None,
            selected_node_id: 0,
        };

        let mut goal_node: Option<Node> = None;
        if JadeSwarm::is_goal(&self.initial_state) {
            goal_node = Some(initial_node)
        } else {
            frontier.push_back(initial_node.clone());

            while let Some(node) = frontier.pop_back() {
                if JadeSwarm::is_goal(&node.state) {
                    goal_node = Some(node);
                    break;
                }
                for child in self.uninformed_expand(&node) {
                        frontier.push_back(child);
                }
            }
        }

        // Trace back path
        JadeSwarm::trace_back_path_with_parent(goal_node)
    } 

/// ---------------------------

    pub fn depth_first_search_with_cycle_checking(&self) -> Option<Vec<Node>> {
        let mut frontier: VecDeque<Node> = VecDeque::new();
        let initial_node = Node {
            state: self.initial_state.clone(),
            cost: 0,
            path_cost: 0,
            parent: None,
            selected_node_id: 0,
        };

        let mut goal_node: Option<Node> = None;
        if JadeSwarm::is_goal(&self.initial_state) {
            goal_node = Some(initial_node)
        } else {
            frontier.push_back(initial_node.clone());

            while let Some(node) = frontier.pop_back() {
                if JadeSwarm::is_goal(&node.state) {
                    goal_node = Some(node);
                    break;
                }
                for child in self.uninformed_expand(&node) {
                    if !JadeSwarm::is_cycle(&child) {
                        frontier.push_back(child);
                    }
                }
            }
        }

        // Trace back path
        JadeSwarm::trace_back_path_with_parent(goal_node)
    } 
}
