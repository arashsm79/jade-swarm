use crate::problem::JadeSwarm;
use crate::problem::Node;
use crate::problem::State;
use crate::problem::StoneColor;

use std::collections::VecDeque;
use std::collections::{HashMap, BinaryHeap};
impl JadeSwarm {
/*
* --------------------------------------
* --------------------------------------
*  Bidirectional Best First Search
* --------------------------------------
* --------------------------------------
*/
    pub fn bidirectional_search(&self) -> Option<Vec<Node>>
    {
        let g = |node: &Node| -> i32 {
            match JadeSwarm::get_color(&node.state, node.selected_node_id) {
                StoneColor::Red => 1,
                StoneColor::Green => 3,
                StoneColor::Black => 2,
            }
        };

        let h = |_node: &Node| -> i32 { 0 };

        let mut reached_f: HashMap<State, Option<Node>> = HashMap::new();
        let mut frontier_f: BinaryHeap<Node> = BinaryHeap::new();
        let initial_node_f = Node {
            state: self.initial_state.clone(),
            cost: 0,
            path_cost: 0,
            selected_node_id: 0,
        };
        frontier_f.push(initial_node_f.clone());
        reached_f.insert(self.initial_state.clone(), None);

        let mut reached_b: HashMap<State, Option<Node>> = HashMap::new();
        let mut frontier_b: BinaryHeap<Node> = BinaryHeap::new();
        let initial_node_b = Node {
            state: State {
                config: vec![true; self.initial_state.config.len()],
                blacks_pos: vec![true; self.initial_state.config.len()],
            },
            cost: 0,
            path_cost: 0,
            selected_node_id: 0,
        };
        frontier_b.push(initial_node_b.clone());
        reached_b.insert(initial_node_b.state.clone(), None);

        let mut solution: Option<Vec<Node>> = None;
        while !frontier_f.is_empty() && !frontier_b.is_empty() && solution == None {
            if frontier_f.peek().unwrap().path_cost < frontier_b.peek().unwrap().path_cost {
                solution = self.bidirectional_search_proceed(
                    true,
                    &mut frontier_f,
                    &mut reached_f,
                    &reached_b,
                    g, h
                );
            } else {
                solution = self.bidirectional_search_proceed(
                    false,
                    &mut frontier_b,
                    &mut reached_b,
                    &reached_f,
                    g, h
                );
            }
        }
        solution
    }

    fn bidirectional_search_proceed<F>(
        &self,
        is_forward: bool,
        frontier1: &mut BinaryHeap<Node>,
        reached1: &mut HashMap<State, Option<Node>>,
        reached2: &HashMap<State, Option<Node>>,
        g: fn(&Node) -> i32, 
        h: F
    ) -> Option<Vec<Node>>
    where
        F: Fn(&Node) -> i32,
    {
        if let Some(node) = frontier1.pop() {
            for child in self.informed_expand(&node, g, &h) {
                if !reached1.contains_key(&child.state)
                    || child.cost
                        < if let Some(node) = reached1
                            .get(&child.state)
                            .unwrap()
                            .as_ref() {
                            node.cost
                        } else {
                            0
                        }
                {
                    reached1.insert(child.state.clone(), Some(node.clone()));
                    if reached2.contains_key(&child.state) {
                        return JadeSwarm::bidirectional_search_join(is_forward, child, reached1, reached2);
                    }
                    frontier1.push(child);
                }
            }
        }
        return None
    }

    fn bidirectional_search_join(
        is_forward: bool,
        intersection_node: Node,
        reached1: &HashMap<State, Option<Node>>,
        reached2: &HashMap<State, Option<Node>>,
    ) -> Option<Vec<Node>> {
        let mut path: VecDeque<Node> = VecDeque::new();

        // Supposing reached1 is forward (if it's not we will reverse the path list)
        // find path from intersection to start of reached1
        path.push_back(intersection_node.clone());
        let mut p = &intersection_node.state;
        loop {
            if let Some(p_unwrapped) = reached1.get(p) {
                // There is a key with p
                match p_unwrapped {
                    Some(parent) => {
                        // Node has a parent
                        path.push_back(parent.clone());
                        p = &parent.state;
                    }
                    None => {
                        // Node doesn'nt a have a parent (we have reached the initial state)
                        break;
                    }
                }
            } else {
                    // There isn't a key with p
                    break;
                }
        }
        // find path from intersection to start of reached2
        let mut p = &intersection_node.state;
        loop {
            if let Some(p_unwrapped) = reached2.get(p) {
                // There is a key with p
                match p_unwrapped {
                    Some(parent) => {
                        // Node has a parent
                        path.push_front(parent.clone());
                        p = &parent.state;
                    }
                    None => {
                        // Node doesn'nt a have a parent (we have reached the initial state)
                        break;
                    }
                }
            } else {
                    // There isn't a key with p
                    break;
                }
        }
        if is_forward {
            Some(Vec::from(path))
        } else {
            Some(Vec::from_iter(path.into_iter().rev()))
        }
    }
    
}
