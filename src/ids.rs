use crate::problem::JadeSwarm;
use crate::problem::Node;
use crate::problem::State;

use std::collections::VecDeque;
use std::collections::HashSet;
impl JadeSwarm {
/*
* --------------------------------------
* --------------------------------------
*  Iterative Deepening Search
* --------------------------------------
* --------------------------------------
*/
    fn depth_limited_with_reached(&self, depth: usize) -> Option<Vec<Node>> {
        // Do the search
        let mut reached: HashSet<State> = HashSet::new();
        let mut frontier: VecDeque<(Node, usize)> = VecDeque::new();
        let initial_node = Node {
            state: self.initial_state.clone(),
            cost: 0,
            parent: None,
            path_cost: 0,
            selected_node_id: 0,
        };

        let mut goal_node: Option<Node> = None;
        if JadeSwarm::is_goal(&self.initial_state) {
            goal_node = Some(initial_node)
        } else {
            frontier.push_back((initial_node.clone(), 0));
            reached.insert(initial_node.state);

            while let Some((node, l)) = frontier.pop_back() {
                if JadeSwarm::is_goal(&node.state) {
                    goal_node = Some(node);
                    break;
                }

                if l < depth {
                    for child in self.uninformed_expand(&node) {
                        if !reached.contains(&child.state) {
                            reached.insert(child.state.clone());
                            frontier.push_back((child, l + 1));
                        }
                    }
                }
            }
        }

        // Trace back path
        JadeSwarm::trace_back_path_with_parent(goal_node)
    }


    pub fn iterative_deepening_search_with_reached(&self) -> Option<Vec<Node>> {
        for depth in 0..usize::MAX {
            println!("depth {}", depth);
            match self.depth_limited_with_reached(depth) {
                Some(path) => return Some(path),
                None => continue,
            }
        }
        return None;
    }    

/// ---------------------------

    fn depth_limited_without_cycle_checking(&self, depth: usize) -> Option<Vec<Node>> {
        // Do the search
        let mut frontier: VecDeque<(Node, usize)> = VecDeque::new();
        let initial_node = Node {
            state: self.initial_state.clone(),
            cost: 0,
            parent: None,
            path_cost: 0,
            selected_node_id: 0,
        };

        let mut goal_node: Option<Node> = None;
        if JadeSwarm::is_goal(&self.initial_state) {
            goal_node = Some(initial_node)
        } else {
            frontier.push_back((initial_node.clone(), 0));

            while let Some((node, l)) = frontier.pop_back() {
                if JadeSwarm::is_goal(&node.state) {
                    goal_node = Some(node);
                    break;
                }

                if l < depth {
                    for child in self.uninformed_expand(&node) {
                        frontier.push_back((child, l + 1));
                    }
                }
            }
        }

        // Trace back path
        JadeSwarm::trace_back_path_with_parent(goal_node)
    }

    pub fn iterative_deepening_search_without_cycle_checking(&self) -> Option<Vec<Node>> {
        for depth in 0..usize::MAX {
            println!("depth {}", depth);
            match self.depth_limited_without_cycle_checking(depth) {
                Some(path) => return Some(path),
                None => continue,
            }
        }
        return None;
    }    

/// ---------------------------

    fn depth_limited_with_cycle_checking(&self, depth: usize) -> Option<Vec<Node>> {
        // Do the search
        let mut frontier: VecDeque<(Node, usize)> = VecDeque::new();
        let initial_node = Node {
            state: self.initial_state.clone(),
            cost: 0,
            parent: None,
            path_cost: 0,
            selected_node_id: 0,
        };

        let mut goal_node: Option<Node> = None;
        if JadeSwarm::is_goal(&self.initial_state) {
            goal_node = Some(initial_node)
        } else {
            frontier.push_back((initial_node.clone(), 0));

            while let Some((node, l)) = frontier.pop_back() {
                if JadeSwarm::is_goal(&node.state) {
                    goal_node = Some(node);
                    break;
                }

                if l < depth {
                    for child in self.uninformed_expand(&node) {
                        if !JadeSwarm::is_cycle(&child) {
                            frontier.push_back((child, l + 1));
                        }
                    }
                }
            }
        }

        // Trace back path
        JadeSwarm::trace_back_path_with_parent(goal_node)
    }

    pub fn iterative_deepening_search_with_cycle_checking(&self) -> Option<Vec<Node>> {
        for depth in 0..usize::MAX {
            println!("depth {}", depth);
            match self.depth_limited_with_cycle_checking(depth) {
                Some(path) => return Some(path),
                None => continue,
            }
        }
        return None;
    }    
}
