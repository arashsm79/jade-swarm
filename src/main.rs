use bitvec::prelude::*;
use std::collections::{HashMap, VecDeque};

struct JadeSwarm {
    graph: JadeGraph,
    initial_state: State,
}

#[derive(Debug)]
struct JadeGraph {
    size: usize,
    adj_mat: Vec<BitVec<Lsb0, u8>>,
    initial_config: BitVec<Lsb0, u8>,
    initial_blacks_pos: BitVec<Lsb0, u8>,
}

#[derive(Clone, PartialEq, Eq, Hash, Debug)]
struct State {
    config: BitVec<Lsb0, u8>,
    blacks_pos: BitVec<Lsb0, u8>,
    selected_node: usize, // which node was selected that lead to this state
}

enum StoneColor {
    Green,
    Red,
    Black
}

impl JadeSwarm {

    fn new(graph: JadeGraph) -> JadeSwarm {
        JadeSwarm {
            initial_state: State {
                config: graph.initial_config.clone(),
                blacks_pos: graph.initial_blacks_pos.clone(),
                selected_node: 0
            },
            graph,
        }
    }

    // Returns the color of a node in the given state
    fn get_color(currenct_state: &State, selected_node: usize) -> StoneColor {
        if currenct_state.config[selected_node] == true {
            StoneColor::Green
        } else if currenct_state.blacks_pos[selected_node] == false {
            StoneColor::Black
        } else {
            StoneColor::Red
        }
    }

    fn successor(&self, current_state: &State, selected_node: usize) -> State {
        let mut config = current_state.config.clone();
        let mut blacks_pos = current_state.blacks_pos.clone();
        let graph = &self.graph;

        // TODO find a way to avoid cloning
        match JadeSwarm::get_color(current_state, selected_node) {
            StoneColor::Red => {
                config = (config ^ graph.adj_mat[selected_node].clone()) & current_state.blacks_pos.clone();
            },
            StoneColor::Green => {
                config = (config ^ graph.adj_mat[selected_node].clone()) & current_state.blacks_pos.clone();
            },
            StoneColor::Black => {
                config = (config ^ graph.adj_mat[selected_node].clone()) & current_state.blacks_pos.clone();
                let neighbors_count = (graph.adj_mat[selected_node].clone() & current_state.blacks_pos.clone())
                    .iter().filter(|b| (*b)==true).count();
                let green_neighbors_count = (config.clone() & (graph.adj_mat[selected_node].clone() & current_state.blacks_pos.clone()))
                    .iter().filter(|b| (*b)==true).count();
                let red_neighbors_count = neighbors_count - green_neighbors_count;
                if green_neighbors_count > neighbors_count / 2 { // Turn black node to green
                    config.set(selected_node, true);
                    blacks_pos.set(selected_node, true);
                } else if red_neighbors_count > neighbors_count / 2 { // Turn black node to red
                    config.set(selected_node, false);
                    blacks_pos.set(selected_node, false);
                } // else the black node stays black
            },
        }
        State { config, blacks_pos, selected_node}
    }

    fn is_goal(state: &State) -> bool {
       state.config.iter().filter(|b| (*b)==true).count() == state.config.len()
    }

    fn breadth_first_search(&self) -> Option<Vec<State>> {

        // Do the search
        let mut reached : HashMap<State, Option<State>> = HashMap::new();
        let mut frontier : VecDeque<State> = VecDeque::new();
        let initial_node = self.initial_state.clone();

        let mut goal_node : Option<State> = None;
        if JadeSwarm::is_goal(&self.initial_state) {
            goal_node = Some(initial_node)
        } else {
            frontier.push_back(initial_node.clone());
            reached.insert(initial_node, None);
        }

        while !frontier.is_empty() {
            let node_wrapped = frontier.pop_front();
            match node_wrapped {
                Some(node) => {
                    for child in self.bfs_expand(&node) {
                        if JadeSwarm::is_goal(&child) {
                            goal_node = Some(child.clone());
                            reached.insert(child, Some(node));
                            break
                        }
                        if !reached.contains_key(&child) {
                            frontier.push_back(child.clone());
                            reached.insert(child, Some(node.clone()));
                        }
                    }
                },
                None => {
                    break
                }
            }
        }

        // Trace back path
        match goal_node {
            Some(goal_node_unwrapped) => { // The goal state was found
                let mut path : Vec<State> = Vec::new();
                path.push(goal_node_unwrapped.clone());
                let mut p = &goal_node_unwrapped;
                loop {
                    if let Some(p_unwrapped) = reached.get(p) { // There is a key with p
                        match p_unwrapped {
                            Some(parent) => { // Node has a parent
                                p = parent;
                                path.push(p.clone());
                            },
                            None => { // Node doesn'nt a have a parent (we have reached the initial state)
                                break
                            }
                        }
                    } else { // There isn't a key with p
                        break
                    }
                }
                return Some(path)
            },
            None => { // No solutions were found
                return None
            }
        }
    }

    fn bfs_expand(&self, state: &State) -> Vec<State> {
        let mut expanded_nodes = Vec::new();
        for i in 0..state.config.len() {
           expanded_nodes.push(self.successor(state, i))
        }
        expanded_nodes
    }


    fn depth_first_search(&self) -> Option<Vec<State>> {
        return None
    }

    fn write_results(&self, path : Vec<State>) -> String {
        let mut output = String::new();
        for p in path.iter().rev() {
            output.push_str(&p.selected_node.to_string());
            output.push_str(" ,");

            for node_index in 0..p.config.len() {
                let color = match JadeSwarm::get_color(&p, node_index) {
                    StoneColor::Red => 'R',
                    StoneColor::Green => 'G',
                    StoneColor::Black => 'B',
                };

                output.push_str(&node_index.to_string());
                output.push(color);
                output.push(' ');

                let neighbors_row = &self.graph.adj_mat[node_index];
                for (neighbor_index, neighbor_value) in neighbors_row.iter().enumerate() {
                    if *neighbor_value && node_index != neighbor_index {
                        let color = match JadeSwarm::get_color(&p, neighbor_index) {
                            StoneColor::Red => 'R',
                            StoneColor::Green => 'G',
                            StoneColor::Black => 'B',
                        };

                        output.push_str(&neighbor_index.to_string());
                        output.push(color);
                        output.push(' ');

                    }
                }
                output.push_str(",");
            }
            output.push('\n');
        }
        output
    }
}


impl JadeGraph {

    fn size(&self) -> usize {
        self.size
    }

    fn new(size : usize) -> JadeGraph {

        let mut adj_mat : Vec<BitVec<Lsb0, u8>> = Vec::new();
        for i in 0..size {
            let mut row = bitvec![Lsb0, u8; 0; size]; 
            row.set(i, true);
            adj_mat.push(row);
        }
        
        JadeGraph { 
            size,
            adj_mat,
            initial_config: bitvec![Lsb0, u8; 0; size],
            initial_blacks_pos: bitvec![Lsb0, u8; 1; size],
        }
    }

    fn set_edge(&mut self, edge: (usize, usize)) {
        let size = self.size();
        assert!(edge.0 < size, "The first node index is out of range.");
        assert!(edge.1 < size, "The second node index is out of range.");
        assert!(edge.0 != edge.1, "Loops are not allowed.");
        self.adj_mat[edge.0].set(edge.1, true);
        self.adj_mat[edge.1].set(edge.0, true);
    }

    fn set_node_color(&mut self, node_number: usize, color: StoneColor) {
        assert!(node_number < self.size(), "Can't color node. Node out of bounds");
        match color {
            StoneColor::Red => {
                self.initial_config.set(node_number, false);
            },
            StoneColor::Green => {
                self.initial_config.set(node_number, true);
            },
            StoneColor::Black => {
                self.initial_config.set(node_number, false);
                self.initial_blacks_pos.set(node_number, false);
            },
        }

    }
}

fn main() {
    test1();
}

fn test2() {
    let mut jade_graph = JadeGraph::new(7);
    jade_graph.set_edge((0, 4));
    jade_graph.set_edge((1, 2));
    jade_graph.set_edge((1, 3));
    jade_graph.set_edge((1, 4));
    jade_graph.set_edge((3, 5));
    jade_graph.set_edge((4, 5));
    jade_graph.set_edge((5, 6));

    jade_graph.set_node_color(0, StoneColor::Red);
    jade_graph.set_node_color(1, StoneColor::Black);
    jade_graph.set_node_color(2, StoneColor::Green);
    jade_graph.set_node_color(3, StoneColor::Red);
    jade_graph.set_node_color(4, StoneColor::Red);
    jade_graph.set_node_color(5, StoneColor::Green);
    jade_graph.set_node_color(6, StoneColor::Red);
    let jade_swarm = JadeSwarm::new(jade_graph);

    println!("Test 2 Solutions: ");
    println!("BFS: ");
    match jade_swarm.breadth_first_search() {
        Some(path) => {
            println!("{}", jade_swarm.write_results(path))
        },
        None => {
            println!("No solutions were found.")
        }
    }
}

fn test1() {
    let mut jade_graph = JadeGraph::new(5);
    jade_graph.set_edge((0, 1));
    jade_graph.set_edge((0, 2));
    jade_graph.set_edge((0, 3));
    jade_graph.set_edge((1, 2));
    jade_graph.set_edge((3, 4));

    jade_graph.set_node_color(0, StoneColor::Red);
    jade_graph.set_node_color(1, StoneColor::Red);
    jade_graph.set_node_color(2, StoneColor::Red);
    jade_graph.set_node_color(3, StoneColor::Green);
    jade_graph.set_node_color(4, StoneColor::Red);
    let jade_swarm = JadeSwarm::new(jade_graph);

    println!("Test 2 Solutions: ");
    println!("BFS: ");
    match jade_swarm.breadth_first_search() {
        Some(path) => {
            println!("{}", jade_swarm.write_results(path))
        },
        None => {
            println!("No solutions were found.")
        }
    }
}

fn test3() {
    let mut jade_graph = JadeGraph::new(15);
    jade_graph.set_edge((0, 1));
    jade_graph.set_edge((0, 2));
    jade_graph.set_edge((1, 14));
    jade_graph.set_edge((1, 2));
    jade_graph.set_edge((1, 3));
    jade_graph.set_edge((2, 5));
    jade_graph.set_edge((2, 6));
    jade_graph.set_edge((2, 7));
    jade_graph.set_edge((3, 13));
    jade_graph.set_edge((3, 14));
    jade_graph.set_edge((3, 7));
    jade_graph.set_edge((4, 6));
    jade_graph.set_edge((4, 11));
    jade_graph.set_edge((5, 10));
    jade_graph.set_edge((5, 12));
    jade_graph.set_edge((6, 11));
    jade_graph.set_edge((7, 8));
    jade_graph.set_edge((7, 9));
    jade_graph.set_edge((8, 14));

    jade_graph.set_node_color(0, StoneColor::Red);
    jade_graph.set_node_color(1, StoneColor::Black);
    jade_graph.set_node_color(2, StoneColor::Black);
    jade_graph.set_node_color(3, StoneColor::Black);
    jade_graph.set_node_color(4, StoneColor::Red);
    jade_graph.set_node_color(5, StoneColor::Green);
    jade_graph.set_node_color(6, StoneColor::Green);
    jade_graph.set_node_color(7, StoneColor::Red);
    jade_graph.set_node_color(8, StoneColor::Red);
    jade_graph.set_node_color(9, StoneColor::Green);
    jade_graph.set_node_color(10, StoneColor::Red);
    jade_graph.set_node_color(11, StoneColor::Red);
    jade_graph.set_node_color(12, StoneColor::Red);
    jade_graph.set_node_color(13, StoneColor::Green);
    jade_graph.set_node_color(14, StoneColor::Red);
    let jade_swarm = JadeSwarm::new(jade_graph);

    println!("Test 3 Solutions: ");
    println!("BFS: ");
    match jade_swarm.breadth_first_search() {
        Some(path) => {
            println!("{}", jade_swarm.write_results(path))
        },
        None => {
            println!("No solutions were found.")
        }
    }
}
