use bitvec::prelude::*;
use std::collections::hash_map::DefaultHasher;
use core::hash::Hasher;
use core::hash::Hash;
use std::collections::{HashMap, VecDeque};
use std::cmp::Ordering;
use std::collections::BinaryHeap;

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

#[derive(Clone, PartialEq, Eq, Debug)]
struct State {
    config: BitVec<Lsb0, u8>,
    blacks_pos: BitVec<Lsb0, u8>,
    selected_node: usize, // which node was selected that lead to this state
}

impl Hash for State {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.config.hash(state);
        self.blacks_pos.hash(state);
    }
}

// TODO make sure Ord is consistent with PartialOrd
#[derive(Clone, PartialEq, Eq, Hash, Debug)]
struct Node {
    state: State,
    cost: i32
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
                selected_node: 0,
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
        config = (config ^ graph.adj_mat[selected_node].clone()) & current_state.blacks_pos.clone();
        match JadeSwarm::get_color(current_state, selected_node) {
            StoneColor::Red => {
            },
            StoneColor::Green => {
            },
            StoneColor::Black => {
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

    fn a_star(&self) -> Option<Vec<State>> {

        let g = |_state: &State| -> i32 {
            1
        };

        // Consistent and admissible heuristic:
        /*
        * Negative of ((number of non-green nodes) / (maximum branching factor of graph))
        */
        let mut maximum_branching_factor = 0;
        for adj_mat_row in &self.graph.adj_mat {
            maximum_branching_factor = std::cmp::max(maximum_branching_factor, adj_mat_row.iter().filter(|b| (*b)==true).count());
        }

        let h = |state: &State| -> i32 {
            let x = f64::from(state.config.len() as i32 - state.config.iter().filter(|b| (*b)==true).count() as i32) / maximum_branching_factor as f64;
            x.ceil() as i32
        };

        self.best_first_search(g, h)
    }

    fn greedy_best_first_search(&self) -> Option<Vec<State>> {

        let g = |_state: &State| -> i32 {
            0
        };

        // Consistent and admissible heuristic:
        /*
        * Negative of ((number of non-green nodes) / (maximum branching factor of graph))
        */
        let mut maximum_branching_factor = 0;
        for adj_mat_row in &self.graph.adj_mat {
            maximum_branching_factor = std::cmp::max(maximum_branching_factor, adj_mat_row.iter().filter(|b| (*b)==true).count());
        }

        let h = |state: &State| -> i32 {
            ((state.config.len() as i32) - (state.config.iter().filter(|b| (*b)==true).count()) as i32) / maximum_branching_factor as i32
        };

        self.best_first_search(g, h)
    }

    fn uniform_cost_search(&self) -> Option<Vec<State>> {

        let g = |state: &State| -> i32 {
            match JadeSwarm::get_color(&state, state.selected_node) {
                StoneColor::Red =>  1,
                StoneColor::Green =>  3,
                StoneColor::Black =>  2,
            }
        };

        let h = |_state: &State| -> i32 {
            0
        };

        self.best_first_search(g, h)
    }

    fn best_first_search<F>(&self, g: fn(&State) -> i32, h: F) -> Option<Vec<State>>
    where F: Fn(&State) -> i32 {
        let mut reached : HashMap<State, (Option<State>, Node)> = HashMap::new();
        let mut frontier : BinaryHeap<Node> = BinaryHeap::new();
        let initial_node = Node { state: self.initial_state.clone(), cost: 0};
        let mut goal_node : Option<Node> = None;

        frontier.push(initial_node.clone());
        reached.insert(self.initial_state.clone(), (None, initial_node));

        while let Some(node) = frontier.pop() {

            if JadeSwarm::is_goal(&node.state) {
                goal_node = Some(node);
                break
            }

            for child in self.best_first_expand(&node, g, &h) {
                if !reached.contains_key(&child.state) || child.cost < reached.get(&child.state).unwrap().1.cost  {
                    reached.insert(child.state.clone(), (Some(node.state.clone()), child.clone()));
                    frontier.push(child);
                }
            }
        }

        // Trace back path
        JadeSwarm::best_first_trace_back_path(goal_node, reached)
    }

    fn breadth_first_search(&self) -> Option<Vec<State>> {
        let mut count = 0;

        // Do the search
        let mut reached : HashMap<u64, Option<State>> = HashMap::new();
        let mut frontier : VecDeque<State> = VecDeque::new();
        let initial_node = self.initial_state.clone();

        let mut goal_node : Option<State> = None;
        if JadeSwarm::is_goal(&self.initial_state) {
            goal_node = Some(initial_node)
        } else {
            frontier.push_back(initial_node.clone());
            println!("added child: {:?}", initial_node);
            println!("with hash child: {:?}\n", calculate_hash(&initial_node));
            reached.insert(calculate_hash(&initial_node), None);
        }

        while let Some(node) = frontier.pop_front() {
            println!("Expansion {} node: {:?}\n", count, node);
            for child in self.expand(&node) {
                println!("child node: {:?}\n", child);
                if JadeSwarm::is_goal(&child) {
                    goal_node = Some(child.clone());
                    reached.insert(calculate_hash(&child), Some(node));
                    break
                }
                if !reached.contains_key(&calculate_hash(&child)) {
                    println!("added child: {:?}", child);
                    println!("with hash child: {:?}\n", calculate_hash(&child));
                    frontier.push_back(child.clone());
                    reached.insert(calculate_hash(&child), Some(node.clone()));
                }
            }
            count = count + 1;
        }

        // Trace back path
        // JadeSwarm::trace_back_path(goal_node, reached)
        None
    }

    fn expand(&self, state: &State) -> Vec<State> {
        let mut expanded_nodes = Vec::new();
        for i in 0..state.config.len() {
            expanded_nodes.push(self.successor(state, i))
        }
        expanded_nodes
    }

    fn best_first_expand<F>(&self, node: &Node, g: fn(&State) -> i32, h: F) -> Vec<Node>
        where F: Fn(&State) -> i32 {
        let mut expanded_nodes = Vec::new();
        for i in 0..node.state.config.len() {
            let next_state = self.successor(&node.state, i);
           expanded_nodes.push(
                Node {
                    cost: node.cost + g(&next_state) + h(&next_state),
                    state: next_state,
                }
            )
        }
        expanded_nodes
    }

    fn depth_first_search(&self) -> Option<Vec<State>> {
        let mut reached : HashMap<State, Option<State>> = HashMap::new();
        let mut frontier : VecDeque<(State, Option<State>)> = VecDeque::new();
        let initial_node = self.initial_state.clone();

        let mut goal_node : Option<State> = None;
        if JadeSwarm::is_goal(&self.initial_state) {
            goal_node = Some(initial_node)
        } else {
            frontier.push_back((initial_node.clone(), None));

            while let Some((node, previous_node)) = frontier.pop_back() {
                reached.insert(node.clone(), previous_node);
                if JadeSwarm::is_goal(&node) {
                    goal_node = Some(node);
                    break
                }
                for child in self.expand(&node) {
                    if !reached.contains_key(&child) {
                        frontier.push_back((child, Some(node.clone())));
                    }
                }
            }
        }

        // Trace back path
        JadeSwarm::trace_back_path(goal_node, reached)
    }

    fn trace_back_path(goal_node: Option<State>, reached: HashMap<State, Option<State>>) -> Option<Vec<State>> {
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

    fn best_first_trace_back_path(goal_node: Option<Node>, reached: HashMap<State, (Option<State>, Node)>) -> Option<Vec<State>> {
        match goal_node {
            Some(goal_node_unwrapped) => { // The goal state was found
                let mut path : Vec<State> = Vec::new();
                path.push(goal_node_unwrapped.state.clone());
                let mut p = &goal_node_unwrapped.state;
                loop {
                    if let Some(p_unwrapped) = reached.get(p) { // There is a key with p
                        match p_unwrapped {
                            (Some(parent), _) => { // Node has a parent
                                p = parent;
                                path.push(p.clone());
                            },
                            (None, _) => { // Node doesn'nt a have a parent (we have reached the initial state)
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

    fn depth_limited(&self, depth: usize) -> Option<Vec<State>> {
        // Do the search
        let mut reached : HashMap<State, Option<State>> = HashMap::new();
        let mut frontier : VecDeque<(State, usize)> = VecDeque::new();
        let initial_node = self.initial_state.clone();

        let mut goal_node : Option<State> = None;
        if JadeSwarm::is_goal(&self.initial_state) {
            goal_node = Some(initial_node)
        } else {
            frontier.push_back((initial_node.clone(), 0));
            reached.insert(initial_node, None);

            while let Some((node, l)) = frontier.pop_back() {
                if JadeSwarm::is_goal(&node) {
                    goal_node = Some(node);
                    break
                }

                if l < depth {
                    for child in self.expand(&node) {
                        if !reached.contains_key(&child) {
                            reached.insert(child.clone(), Some(node.clone()));
                            frontier.push_back((child, l+1));
                        }
                    }
                }
            }
        }

        // Trace back path
        JadeSwarm::trace_back_path(goal_node, reached)
    }

    fn iterative_deepening_search(&self) -> Option<Vec<State>> {
        for depth in 0..usize::MAX {
            println!("depth {}", depth);
            match self.depth_limited(depth) {
                Some(path) => return Some(path),
                None => continue,
            }
        }
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

enum SearchMethod {
    BFS(bool),
    DFS(bool),
    IDA(bool),
    UCS(bool),
    BidiS(bool),
    AStar(bool),
    GBFS(bool),
    RBFS(bool),
    IDAStar(bool)
}

fn main() {
    let search_choices = vec![
        SearchMethod::BFS(true),
        SearchMethod::DFS(false),
        SearchMethod::IDA(true),
        SearchMethod::UCS(false),
        SearchMethod::BidiS(false),
        SearchMethod::AStar(false),
        SearchMethod::GBFS(false),
        SearchMethod::RBFS(false),
        SearchMethod::IDAStar(false)
    ];
    // test1(&search_choices);
    // test2(&search_choices);
    test1(&search_choices);
}

fn test2(search_choices: &Vec<SearchMethod>) {
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
    run_search_choices(search_choices, &jade_swarm)
}

fn calculate_hash<T: Hash>(t: &T) -> u64 {
    let mut s = DefaultHasher::new();
    t.hash(&mut s);
    s.finish()
}

fn run_search_choices(search_choices: &Vec<SearchMethod>, jade_swarm: &JadeSwarm) {
    for search_choice in search_choices {
        match search_choice {
            SearchMethod::BFS(true) => {
                println!("BFS: ");
                match jade_swarm.breadth_first_search() {
                    Some(path) => {
                        for elem in &path {
                            println!("{:#?}", calculate_hash(elem));
                        }
                        println!("{}", jade_swarm.write_results(path))
                    },
                    None => {
                        println!("No BFS solutions were found.")
                    }
                }
            },
            SearchMethod::DFS(true) => {
                println!("DFS: ");
                match jade_swarm.depth_first_search() {
                    Some(path) => {
                        println!("{}", jade_swarm.write_results(path))
                    },
                    None => {
                        println!("No DFS solutions were found.")
                    }
                }
            },
            SearchMethod::IDA(true) => {
                println!("IDS: ");
                match jade_swarm.iterative_deepening_search() {
                    Some(path) => {
                        for elem in &path {
                            println!("{:#?}", calculate_hash(elem));
                        }
                        println!("{}", jade_swarm.write_results(path))
                    },
                    None => {
                        println!("No IDA solutions were found.")
                    }
                }
            },
            SearchMethod::UCS(true) => {
                println!("UCS: ");
                match jade_swarm.uniform_cost_search() {
                    Some(path) => {
                        println!("{}", jade_swarm.write_results(path))
                    },
                    None => {
                        println!("No UCS solutions were found.")
                    }
                }
            },
            SearchMethod::BidiS(true) => {
            },
            SearchMethod::AStar(true) => {
                println!("A*: ");
                match jade_swarm.a_star() {
                    Some(path) => {
                        println!("{}", jade_swarm.write_results(path))
                    },
                    None => {
                        println!("No A* solutions were found.")
                    }
                }
            },
            SearchMethod::GBFS(true) => {
                println!("GBFS: ");
                match jade_swarm.greedy_best_first_search() {
                    Some(path) => {
                        println!("{}", jade_swarm.write_results(path))
                    },
                    None => {
                        println!("No GBFS solutions were found.")
                    }
                }
            },
            SearchMethod::RBFS(true) => {
            },
            SearchMethod::IDAStar(true) => {
            },
            _ => ()
        }
    }
}

fn test1(search_choices: &Vec<SearchMethod>) {
    let mut jade_graph = JadeGraph::new(5);
    jade_graph.set_edge((0, 1));
    jade_graph.set_edge((0, 2));
    jade_graph.set_edge((0, 3));
    jade_graph.set_edge((1, 2));
    jade_graph.set_edge((3, 4));

    jade_graph.set_node_color(0, StoneColor::Black);
    jade_graph.set_node_color(1, StoneColor::Red);
    jade_graph.set_node_color(2, StoneColor::Red);
    jade_graph.set_node_color(3, StoneColor::Green);
    jade_graph.set_node_color(4, StoneColor::Red);
    let jade_swarm = JadeSwarm::new(jade_graph);

    println!("Test 1 Solutions: ");
    run_search_choices(search_choices, &jade_swarm)
}


fn test3(search_choices: &Vec<SearchMethod>) {
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
    run_search_choices(search_choices, &jade_swarm)
}
