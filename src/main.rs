use bitvec::prelude::*;
use core::hash::Hash;
use core::hash::Hasher;
use std::cmp::Ordering;
use std::collections::hash_map::DefaultHasher;
use std::collections::BinaryHeap;
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
}

// TODO make sure Ord is consistent with PartialOrd
#[derive(Clone, PartialEq, Eq, Hash, Debug)]
struct Node {
    state: State,
    selected_node_id: usize, // which node on the previous state was selected that lead to this node with this state
    path_cost: i32,
    cost: i32,
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
    Black,
}

impl JadeSwarm {
    fn new(graph: JadeGraph) -> JadeSwarm {
        JadeSwarm {
            initial_state: State {
                config: graph.initial_config.clone(),
                blacks_pos: graph.initial_blacks_pos.clone(),
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
            StoneColor::Red => {}
            StoneColor::Green => {}
            StoneColor::Black => {
                let neighbors_count = (graph.adj_mat[selected_node].clone()
                    & current_state.blacks_pos.clone())
                .iter()
                .filter(|b| (*b) == true)
                .count();
                let green_neighbors_count = (config.clone()
                    & (graph.adj_mat[selected_node].clone() & current_state.blacks_pos.clone()))
                .iter()
                .filter(|b| (*b) == true)
                .count();
                let red_neighbors_count = neighbors_count - green_neighbors_count;
                if green_neighbors_count > neighbors_count / 2 {
                    // Turn black node to green
                    config.set(selected_node, true);
                    blacks_pos.set(selected_node, true);
                } else if red_neighbors_count > neighbors_count / 2 {
                    // Turn black node to red
                    config.set(selected_node, false);
                    blacks_pos.set(selected_node, true);
                } // else the black node stays black
            }
        }
        State { config, blacks_pos }
    }

    fn is_goal(state: &State) -> bool {
        state.config.iter().filter(|b| (*b) == true).count() == state.config.len()
    }

    fn a_star(&self) -> Option<Vec<Node>> {
        let g = |_node: &Node| -> i32 { 1 };

        // Consistent and admissible heuristic:
        /*
         * ((number of non-green nodes) / (maximum branching factor of graph))
         */
        let mut maximum_branching_factor = 0;
        for adj_mat_row in &self.graph.adj_mat {
            maximum_branching_factor = std::cmp::max(
                maximum_branching_factor,
                adj_mat_row.iter().filter(|b| (*b) == true).count(),
            );
        }

        let h = |node: &Node| -> i32 {
            let x = f64::from(
                node.state.config.len() as i32
                    - node.state.config.iter().filter(|b| (*b) == true).count() as i32,
            ) / maximum_branching_factor as f64;
            x.ceil() as i32
        };

        self.best_first_search(g, h)
    }

    fn iterative_deepening_a_star(&self) -> Option<Vec<Node>> {
        let g = |_node: &Node| -> i32 { 1 };
        let mut maximum_branching_factor = 0;
        for adj_mat_row in &self.graph.adj_mat {
            maximum_branching_factor = std::cmp::max(
                maximum_branching_factor,
                adj_mat_row.iter().filter(|b| (*b) == true).count(),
            );
        }
        let h = |node: &Node| -> i32 {
            let x = f64::from(
                node.state.config.len() as i32
                    - node.state.config.iter().filter(|b| (*b) == true).count() as i32,
            ) / maximum_branching_factor as f64;
            x.ceil() as i32
        };

        let initial_node = Node {
            state: self.initial_state.clone(),
            cost: 0,
            path_cost: 0,
            selected_node_id: 0,
        };

        let mut smallest_node_cost = h(&initial_node);

        if JadeSwarm::is_goal(&self.initial_state) {
            return Some(vec![initial_node])
        }

        loop {
            println!("cutoff: {}", smallest_node_cost);
            match self.depth_limited_a_star(smallest_node_cost, g, h) {
                (Some(path), _) => return Some(path),
                (None, cutoff) => smallest_node_cost = cutoff,
            }
        }
    }

    fn depth_limited_a_star<F>(&self, cutoff: i32, g: fn(&Node) -> i32, h: F) -> (Option<Vec<Node>>, i32)
    where
        F: Fn(&Node) -> i32,
    {
        let mut frontier: VecDeque<Node> = VecDeque::new();
        let mut reached: HashMap<State, Option<Node>> = HashMap::new();
        let initial_node = Node {
            state: self.initial_state.clone(),
            cost: 0,
            path_cost: 0,
            selected_node_id: 0,
        };
        let mut goal_node: Option<Node> = None;
        let mut smallest_node_cost = i32::MAX;
        frontier.push_back(initial_node.clone());
        reached.insert(self.initial_state.clone(), None);

        while let Some(node) = frontier.pop_back() {
            if JadeSwarm::is_goal(&node.state) {
                goal_node = Some(node);
                break;
            }

            for child in self.best_first_expand(&node, g, &h) {
                if child.cost <= cutoff {
                    if !reached.contains_key(&child.state)
                        || child.cost
                            < if let Some(node) = reached
                                .get(&child.state)
                                .unwrap()
                                .as_ref() {
                                node.cost
                            } else {
                                0
                            }
                    {
                        reached.insert(child.state.clone(), Some(node.clone()));
                        frontier.push_back(child);
                    }
                } else {
                    smallest_node_cost = std::cmp::min(smallest_node_cost, child.cost);
                }
            }
        }

        (JadeSwarm::trace_back_path(goal_node, reached), smallest_node_cost)
    }



    fn greedy_best_first_search(&self) -> Option<Vec<Node>> {
        let g = |_node: &Node| -> i32 { 0 };

        // Consistent and admissible heuristic:
        /*
         * Negative of ((number of non-green nodes) / (maximum branching factor of graph))
         */
        let mut maximum_branching_factor = 0;
        for adj_mat_row in &self.graph.adj_mat {
            maximum_branching_factor = std::cmp::max(
                maximum_branching_factor,
                adj_mat_row.iter().filter(|b| (*b) == true).count(),
            );
        }

        let h = |node: &Node| -> i32 {
            let x = f64::from(
                node.state.config.len() as i32
                    - node.state.config.iter().filter(|b| (*b) == true).count() as i32,
            ) / maximum_branching_factor as f64;
            x.ceil() as i32
        };

        self.best_first_search(g, h)
    }

    fn uniform_cost_search(&self) -> Option<Vec<Node>> {
        let g = |node: &Node| -> i32 {
            match JadeSwarm::get_color(&node.state, node.selected_node_id) {
                StoneColor::Red => 1,
                StoneColor::Green => 3,
                StoneColor::Black => 2,
            }
        };

        let h = |_node: &Node| -> i32 { 0 };

        self.best_first_search(g, h)
    }

    fn best_first_search<F>(&self, g: fn(&Node) -> i32, h: F) -> Option<Vec<Node>>
    where
        F: Fn(&Node) -> i32,
    {
        let mut reached: HashMap<State, Option<Node>> = HashMap::new();
        let mut frontier: BinaryHeap<Node> = BinaryHeap::new();
        let initial_node = Node {
            state: self.initial_state.clone(),
            cost: 0,
            path_cost: 0,
            selected_node_id: 0,
        };
        let mut goal_node: Option<Node> = None;

        frontier.push(initial_node.clone());
        reached.insert(self.initial_state.clone(), None);

        while let Some(node) = frontier.pop() {
            if JadeSwarm::is_goal(&node.state) {
                goal_node = Some(node);
                break;
            }

            for child in self.best_first_expand(&node, g, &h) {
                // TODO make below condition more readable
                if !reached.contains_key(&child.state)
                    || child.cost
                        < if let Some(node) = reached
                            .get(&child.state)
                            .unwrap()
                            .as_ref() {
                            node.cost
                        } else {
                            0
                        }
                {
                    reached.insert(child.state.clone(), Some(node.clone()));
                    frontier.push(child);
                }
            }
        }

        // Trace back path
        JadeSwarm::trace_back_path(goal_node, reached)
    }

    fn bidirectional_search(&self) -> Option<Vec<Node>>
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
                config: bitvec![Lsb0, u8; 1; self.initial_state.config.len()],
                blacks_pos: bitvec![Lsb0, u8; 1; self.initial_state.config.len()],
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
            for child in self.best_first_expand(&node, g, &h) {
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

        println!("is_forward {} intersect on {:#?}", is_forward, intersection_node);

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


    fn breadth_first_search(&self) -> Option<Vec<Node>> {
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
            goal_node = Some(initial_node)
        } else {
            frontier.push_back(initial_node.clone());
            reached.insert(initial_node.state, None);
        }

        'outer: while let Some(node) = frontier.pop_front() {
            for child in self.expand(&node) {
                if JadeSwarm::is_goal(&child.state) {
                    reached.insert(child.state.clone(), Some(node));
                    goal_node = Some(child);
                    break 'outer;
                }
                if !reached.contains_key(&child.state) {
                    frontier.push_back(child.clone());
                    reached.insert(child.state, Some(node.clone()));
                }
            }
        }

        // Trace back path
        JadeSwarm::trace_back_path(goal_node, reached)
    }

    fn expand(&self, node: &Node) -> Vec<Node> {
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


    fn best_first_expand<F>(&self, node: &Node, g: fn(&Node) -> i32, h: F) -> Vec<Node>
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

    fn depth_first_search(&self) -> Option<Vec<Node>> {
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
                for child in self.expand(&node) {
                    if !reached.contains_key(&child.state) {
                        frontier.push_back((child, Some(node.clone())));
                    }
                }
            }
        }

        // Trace back path
        JadeSwarm::trace_back_path(goal_node, reached)
    }

    fn trace_back_path(
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
                    for child in self.expand(&node) {
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

    fn iterative_deepening_search(&self) -> Option<Vec<Node>> {
        for depth in 0..usize::MAX {
            println!("depth {}", depth);
            match self.depth_limited(depth) {
                Some(path) => return Some(path),
                None => continue,
            }
        }
        return None;
    }

    fn write_results(&self, path: Vec<Node>) -> String {
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

impl JadeGraph {
    fn size(&self) -> usize {
        self.size
    }

    fn new(size: usize) -> JadeGraph {
        let mut adj_mat: Vec<BitVec<Lsb0, u8>> = Vec::new();
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
        assert!(
            node_number < self.size(),
            "Can't color node. Node out of bounds"
        );
        match color {
            StoneColor::Red => {
                self.initial_config.set(node_number, false);
            }
            StoneColor::Green => {
                self.initial_config.set(node_number, true);
            }
            StoneColor::Black => {
                self.initial_config.set(node_number, false);
                self.initial_blacks_pos.set(node_number, false);
            }
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
    IDAStar(bool),
}

fn main() {
    let search_choices = vec![
        SearchMethod::BFS(false),
        SearchMethod::DFS(false),
        SearchMethod::IDA(false),
        SearchMethod::UCS(false),
        SearchMethod::BidiS(false),
        SearchMethod::AStar(false),
        SearchMethod::GBFS(false),
        SearchMethod::RBFS(false),
        SearchMethod::IDAStar(true),
    ];
    // test1(&search_choices);
    // test2(&search_choices);
    test3(&search_choices);
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
                        println!("{}", jade_swarm.write_results(path))
                    }
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
                    }
                    None => {
                        println!("No DFS solutions were found.")
                    }
                }
            },
            SearchMethod::IDA(true) => {
                println!("IDS: ");
                match jade_swarm.iterative_deepening_search() {
                    Some(path) => {
                        println!("{}", jade_swarm.write_results(path))
                    }
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
                    }
                    None => {
                        println!("No UCS solutions were found.")
                    }
                }
            },
            SearchMethod::BidiS(true) => {
                println!("Bidirectional Search: ");
                match jade_swarm.bidirectional_search() {
                    Some(path) => {
                        println!("{}", jade_swarm.write_results(path))
                    }
                    None => {
                        println!("No BidiS solutions were found.")
                    }
                }
            },
            SearchMethod::AStar(true) => {
                println!("A*: ");
                match jade_swarm.a_star() {
                    Some(path) => {
                        println!("{}", jade_swarm.write_results(path))
                    }
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
                    }
                    None => {
                        println!("No GBFS solutions were found.")
                    }
                }
            },
            SearchMethod::RBFS(true) => {
            },
            SearchMethod::IDAStar(true) => {
                println!("IDA*: ");
                match jade_swarm.iterative_deepening_a_star() {
                    Some(path) => {
                        println!("{}", jade_swarm.write_results(path))
                    }
                    None => {
                        println!("No IDA* solutions were found.")
                    }
                }
            },
            _ => (),
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
