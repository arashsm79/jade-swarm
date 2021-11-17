use crate::problem::JadeSwarm;
use crate::problem::Node;
use crate::problem::State;

use std::collections::VecDeque;
use std::collections::HashSet;

impl JadeSwarm {
/*
* --------------------------------------
* --------------------------------------
*  Iterative Deepening A* Search
* --------------------------------------
* --------------------------------------
*/
    pub fn iterative_deepening_a_star_with_reached(&self) -> Option<Vec<Node>> {
        let g = |_node: &Node| -> i32 { 1 };
        let maximum_branching_factor = self.maximum_branching_factor;
        let h = |node: &Node| -> i32 {
            let x = f64::from(
                node.state.config.len() as i32
                    - node.state.config.iter().filter(|&b| (*b) == true).count() as i32,
            ) / maximum_branching_factor as f64;
            x.ceil() as i32
        };

        let initial_node = Node {
            state: self.initial_state.clone(),
            cost: 0,
            path_cost: 0,
            parent: None,
            selected_node_id: 0,
        };

        let mut smallest_node_cost = h(&initial_node);

        if JadeSwarm::is_goal(&self.initial_state) {
            return Some(vec![initial_node])
        }

        loop {
            println!("cutoff: {}", smallest_node_cost);
            match self.depth_limited_a_star_with_reached(smallest_node_cost, g, h) {
                (Some(path), _) => return Some(path),
                (None, cutoff) => smallest_node_cost = cutoff,
            }
        }
    }

    fn depth_limited_a_star_with_reached<F>(&self, cutoff: i32, g: fn(&Node) -> i32, h: F) -> (Option<Vec<Node>>, i32)
    where
        F: Fn(&Node) -> i32,
    {
        let mut frontier: VecDeque<Node> = VecDeque::new();
        let mut reached: HashSet<State> = HashSet::new();
        let initial_node = Node {
            state: self.initial_state.clone(),
            cost: 0,
            path_cost: 0,
            parent: None,
            selected_node_id: 0,
        };
        let mut goal_node: Option<Node> = None;
        let mut smallest_node_cost = i32::MAX;
        frontier.push_back(initial_node.clone());
        reached.insert(self.initial_state.clone());

        while let Some(node) = frontier.pop_back() {
            if JadeSwarm::is_goal(&node.state) {
                goal_node = Some(node);
                break;
            }

            for child in self.informed_expand(&node, g, &h) {
                if child.cost <= cutoff {
                    if !reached.contains(&child.state) {
                        reached.insert(child.state.clone());
                        frontier.push_back(child);
                    }
                } else {
                    smallest_node_cost = std::cmp::min(smallest_node_cost, child.cost);
                }
            }
        }

        (JadeSwarm::trace_back_path_with_parent(goal_node), smallest_node_cost)
    }

/// ---------------------------

    pub fn iterative_deepening_a_star_without_cycle_checking(&self) -> Option<Vec<Node>> {
        let g = |_node: &Node| -> i32 { 1 };
        let maximum_branching_factor = self.maximum_branching_factor;
        let h = |node: &Node| -> i32 {
            let x = f64::from(
                node.state.config.len() as i32
                    - node.state.config.iter().filter(|&b| (*b) == true).count() as i32,
            ) / maximum_branching_factor as f64;
            x.ceil() as i32
        };

        let initial_node = Node {
            state: self.initial_state.clone(),
            cost: 0,
            path_cost: 0,
            parent: None,
            selected_node_id: 0,
        };

        let mut smallest_node_cost = h(&initial_node);

        if JadeSwarm::is_goal(&self.initial_state) {
            return Some(vec![initial_node])
        }

        loop {
            println!("cutoff: {}", smallest_node_cost);
            match self.depth_limited_a_star_without_cycle_checking(smallest_node_cost, g, h) {
                (Some(path), _) => return Some(path),
                (None, cutoff) => smallest_node_cost = cutoff,
            }
        }
    }

    fn depth_limited_a_star_without_cycle_checking<F>(&self, cutoff: i32, g: fn(&Node) -> i32, h: F) -> (Option<Vec<Node>>, i32)
    where
        F: Fn(&Node) -> i32,
    {
        let mut frontier: VecDeque<Node> = VecDeque::new();
        let initial_node = Node {
            state: self.initial_state.clone(),
            cost: 0,
            path_cost: 0,
            parent: None,
            selected_node_id: 0,
        };
        let mut goal_node: Option<Node> = None;
        let mut smallest_node_cost = i32::MAX;
        frontier.push_back(initial_node.clone());

        while let Some(node) = frontier.pop_back() {
            if JadeSwarm::is_goal(&node.state) {
                goal_node = Some(node);
                break;
            }

            for child in self.informed_expand(&node, g, &h) {
                if child.cost <= cutoff {
                    frontier.push_back(child);
                } else {
                    smallest_node_cost = std::cmp::min(smallest_node_cost, child.cost);
                }
            }
        }

        (JadeSwarm::trace_back_path_with_parent(goal_node), smallest_node_cost)
    }

/// ---------------------------

    pub fn iterative_deepening_a_star_with_cycle_checking(&self) -> Option<Vec<Node>> {
        let g = |_node: &Node| -> i32 { 1 };
        let maximum_branching_factor = self.maximum_branching_factor;
        let h = |node: &Node| -> i32 {
            let x = f64::from(
                node.state.config.len() as i32
                    - node.state.config.iter().filter(|&b| (*b) == true).count() as i32,
            ) / maximum_branching_factor as f64;
            x.ceil() as i32
        };

        let initial_node = Node {
            state: self.initial_state.clone(),
            cost: 0,
            path_cost: 0,
            parent: None,
            selected_node_id: 0,
        };

        let mut smallest_node_cost = h(&initial_node);

        if JadeSwarm::is_goal(&self.initial_state) {
            return Some(vec![initial_node])
        }

        loop {
            println!("cutoff: {}", smallest_node_cost);
            match self.depth_limited_a_star_with_cycle_checking(smallest_node_cost, g, h) {
                (Some(path), _) => return Some(path),
                (None, cutoff) => smallest_node_cost = cutoff,
            }
        }
    }

    fn depth_limited_a_star_with_cycle_checking<F>(&self, cutoff: i32, g: fn(&Node) -> i32, h: F) -> (Option<Vec<Node>>, i32)
    where
        F: Fn(&Node) -> i32,
    {
        let mut frontier: VecDeque<Node> = VecDeque::new();
        let initial_node = Node {
            state: self.initial_state.clone(),
            cost: 0,
            path_cost: 0,
            parent: None,
            selected_node_id: 0,
        };
        let mut goal_node: Option<Node> = None;
        let mut smallest_node_cost = i32::MAX;
        frontier.push_back(initial_node.clone());

        while let Some(node) = frontier.pop_back() {
            if JadeSwarm::is_goal(&node.state) {
                goal_node = Some(node);
                break;
            }

            for child in self.informed_expand(&node, g, &h) {
                if child.cost <= cutoff {
                    if !JadeSwarm::is_cycle(&child) {
                        frontier.push_back(child);
                    }
                } else {
                    smallest_node_cost = std::cmp::min(smallest_node_cost, child.cost);
                }
            }
        }

        (JadeSwarm::trace_back_path_with_parent(goal_node), smallest_node_cost)
    }
}
