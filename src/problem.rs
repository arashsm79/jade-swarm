use core::hash::Hash;
use std::rc::Rc;
use std::cmp::Ordering;
use std::collections::HashMap;

impl JadeGraph {
    fn size(&self) -> usize {
        self.size
    }

    pub fn new(size: usize) -> JadeGraph {
        let mut adj_mat: Vec<Vec<bool>> = Vec::new();
        for i in 0..size {
            let mut row = vec![false; size];
            row[i] = true;
            adj_mat.push(row);
        }

        JadeGraph {
            size,
            adj_mat,
            initial_config: vec![false; size],
            initial_blacks_pos: vec![true; size],
        }
    }

    pub fn set_edge(&mut self, edge: (usize, usize)) {
        let size = self.size();
        assert!(edge.0 < size, "The first node index is out of range.");
        assert!(edge.1 < size, "The second node index is out of range.");
        assert!(edge.0 != edge.1, "Loops are not allowed.");
        self.adj_mat[edge.0][edge.1] = true;
        self.adj_mat[edge.1][edge.0] = true;
    }

    pub fn set_node_color(&mut self, node_number: usize, color: StoneColor) {
        assert!(
            node_number < self.size(),
            "Can't color node. Node out of bounds"
        );
        match color {
            StoneColor::Red => {
                self.initial_config[node_number] = false;
            }
            StoneColor::Green => {
                self.initial_config[node_number] = true;
            }
            StoneColor::Black => {
                self.initial_config[node_number] = false;
                self.initial_blacks_pos[node_number] = false;
            }
        }
    }
}

pub struct JadeSwarm {
    pub graph: JadeGraph,
    pub initial_state: State,
    pub maximum_branching_factor: i32
}

#[derive(Debug)]
pub struct JadeGraph {
    size: usize,
    adj_mat: Vec<Vec<bool>>,
    initial_config: Vec<bool>,
    initial_blacks_pos: Vec<bool>,
}

#[derive(Clone, PartialEq, Eq, Hash, Debug)]
pub struct State {
    pub config: Vec<bool>,
    pub blacks_pos: Vec<bool>,
}

// TODO make sure Ord is consistent with PartialOrd
#[derive(Clone, PartialEq, Eq, Hash, Debug)]
pub struct Node {
    pub state: State,
    pub selected_node_id: usize, // which node on the previous state was selected that lead to this node with this state
    pub path_cost: i32,
    pub cost: i32,
}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        other.cost.cmp(&self.cost)
    }
}

impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

#[derive(Clone, PartialEq, Eq, Hash, Debug)]
pub struct RBFSNode {
    pub state: State,
    pub parent: Option<Rc<RBFSNode>>,
    pub selected_node_id: usize, // which node on the previous state was selected that lead to this node with this state
    pub path_cost: i32,
    pub cost: i32,
}

impl Ord for RBFSNode {
    fn cmp(&self, other: &Self) -> Ordering {
        other.cost.cmp(&self.cost)
    }
}

impl PartialOrd for RBFSNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

pub enum StoneColor {
    Green,
    Red,
    Black,
}

impl JadeSwarm {
    pub fn new(graph: JadeGraph) -> JadeSwarm {
        let mut maximum_branching_factor: i32 = 0;
        for adj_mat_row in &graph.adj_mat {
            maximum_branching_factor = std::cmp::max(
                maximum_branching_factor,
                adj_mat_row.iter().filter(|&b| (*b) == true).count() as i32,
            );
        }
        JadeSwarm {
            initial_state: State {
                config: graph.initial_config.clone(),
                blacks_pos: graph.initial_blacks_pos.clone(),
            },
            graph,
            maximum_branching_factor        }
    }

    // Returns the color of a node in the given state
    pub fn get_color(currenct_state: &State, selected_node: usize) -> StoneColor {
        if currenct_state.config[selected_node] == true {
            StoneColor::Green
        } else if currenct_state.blacks_pos[selected_node] == false {
            StoneColor::Black
        } else {
            StoneColor::Red
        }
    }

    fn xor_and(vec1: &Vec<bool>, vec2: &Vec<bool>, vec3: &Vec<bool>) -> Vec<bool> {
        let size = vec1.len();
        let mut result = vec![false; size];
        for i in 0..size {
            result[i] = (vec1[i] ^ vec2[i]) & vec3[i];
        }
        result
    }

    fn neighbors_count(vec1: &Vec<bool>, vec2: &Vec<bool>) -> usize {
        let mut count = 0;
        for i in 0..vec1.len() {
            if vec1[i] & vec2[i] {
                count += 1;
            }
        }
        count
    }

    fn green_neighbors_count(vec1: &Vec<bool>, vec2: &Vec<bool>, vec3: &Vec<bool>) -> usize {
        let mut count = 0;
        for i in 0..vec1.len() {
            if vec1[i] & vec2[i] & vec3[i] {
                count += 1;
            }
        }
        count
    }

    pub fn successor(&self, current_state: &State, selected_node: usize) -> State {
        let graph = &self.graph;

        // TODO find a way to avoid cloning
        let mut blacks_pos = current_state.blacks_pos.clone();
        let mut config = JadeSwarm::xor_and(&current_state.config, &graph.adj_mat[selected_node], &blacks_pos);

        if let StoneColor::Black = JadeSwarm::get_color(current_state, selected_node) {
            let neighbors_count = JadeSwarm::neighbors_count(&graph.adj_mat[selected_node], &blacks_pos);
            let green_neighbors_count = JadeSwarm::green_neighbors_count(&config, &graph.adj_mat[selected_node], &blacks_pos);
            let red_neighbors_count = neighbors_count - green_neighbors_count;
            if green_neighbors_count > neighbors_count / 2 {
                // Turn black node to green
                config[selected_node] = true;
                blacks_pos[selected_node] = true;
            } else if red_neighbors_count > neighbors_count / 2 {
                // Turn black node to red
                config[selected_node] = false;
                blacks_pos[selected_node] = true;
            } // else the black node stays black
        }
        State { config, blacks_pos }
    }

    pub fn uninformed_expand(&self, node: &Node) -> Vec<Node> {
        let mut expanded_nodes = Vec::new();
        for i in 0..node.state.config.len() {
            expanded_nodes.push(Node {
                state: self.successor(&node.state, i),
                cost: 0,
                path_cost: 0,
                selected_node_id: i,
            })
        }
        expanded_nodes
    }

    pub fn informed_expand<F>(&self, node: &Node, g: fn(&Node) -> i32, h: F) -> Vec<Node>
    where
        F: Fn(&Node) -> i32,
    {
        let mut expanded_nodes = Vec::new();
        for i in 0..node.state.config.len() {
            let next_node = Node {
                state: self.successor(&node.state, i),
                cost: 0,
                path_cost: 0,
                selected_node_id: i,
            };
            let path_cost = node.path_cost + g(&next_node);
            expanded_nodes.push(Node {
                path_cost,
                cost: path_cost + h(&next_node),
                ..next_node
            })
        }
        expanded_nodes
    }    

    pub fn is_goal(state: &State) -> bool {
        state.config.iter().filter(|&b| (*b) == true).count() == state.config.len()
    }

    pub fn trace_back_path(
        goal_node: Option<Node>,
        reached: HashMap<State, Option<Node>>,
    ) -> Option<Vec<Node>> {
        match goal_node {
            Some(goal_node_unwrapped) => {
                // The goal state was found
                let mut path: Vec<Node> = Vec::new();
                path.push(goal_node_unwrapped.clone());
                let mut p = &goal_node_unwrapped.state;
                loop {
                    if let Some(p_unwrapped) = reached.get(p) {
                        // There is a key with p
                        match p_unwrapped {
                            Some(parent) => {
                                // Node has a parent
                                path.push(parent.clone());
                                p = &parent.state;
                            }
                            None => {
                                // Node doesn't a have a parent (we have reached the initial state)
                                break;
                            }
                        }
                    } else {
                        // There isn't a key with p
                        break;
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

    pub fn write_results(&self, path: Vec<Node>) -> String {
        let mut output = String::new();
        for node in path.iter().rev() {
            let state = &node.state;
            output.push_str(&node.selected_node_id.to_string());
            output.push_str(" ,");

            for node_index in 0..state.config.len() {
                let color = match JadeSwarm::get_color(&state, node_index) {
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
                        let color = match JadeSwarm::get_color(&state, neighbor_index) {
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
