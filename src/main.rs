mod problem;

mod breadthfs;
mod dfs;
mod ids;
mod bestfs;
mod ucs;
mod bidibestfs;
mod greedybestfs;
mod a_star;
mod ida_star;
mod rbfs;

use crate::problem::JadeSwarm;
use crate::problem::JadeGraph;
use crate::problem::StoneColor;


enum SearchMethod {
    BFS(bool),
    DFSReached(bool),
    DFSWithoutCycleChecking(bool),
    DFSWithCycleChecking(bool),
    IDSReached(bool),
    IDSWithoutCycleChecking(bool),
    IDSWithCycleChecking(bool),
    UCS(bool),
    BidiS(bool),
    AStar(bool),
    GBFS(bool),
    RBFS(bool),
    IDAStarReached(bool),
    IDAStarWithoutCycleChecking(bool),
    IDAStarWithCycleChecking(bool),
}

fn main() {
    let search_choices = vec![
        SearchMethod::BFS(true),
        SearchMethod::DFSReached(true),
        SearchMethod::DFSWithoutCycleChecking(false),
        SearchMethod::DFSWithCycleChecking(false),
        SearchMethod::IDSReached(true),
        SearchMethod::IDSWithoutCycleChecking(false),
        SearchMethod::IDSWithCycleChecking(false),
        SearchMethod::UCS(true),
        SearchMethod::BidiS(true),
        SearchMethod::AStar(true),
        SearchMethod::GBFS(true),
        SearchMethod::RBFS(true),
        SearchMethod::IDAStarReached(true),
        SearchMethod::IDAStarWithoutCycleChecking(false),
        SearchMethod::IDAStarWithCycleChecking(false),
    ];
    test2(&search_choices);
    // test3 has a lot of nodes. Some search algorithms might take an infeasable amoout of time to
    // complete. You can turn them off by changing the value of the corresponding search algorithm
    // from true to false in the search_choices vector.
}

/*
* --------------------------------------
* --------------------------------------
*  Tests
* --------------------------------------
* --------------------------------------
*/

#[allow(dead_code)]
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

#[allow(dead_code)]
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

#[allow(dead_code)]
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
            SearchMethod::DFSReached(true) => {
                println!("DFS With Reached: ");
                match jade_swarm.depth_first_search_with_reached() {
                    Some(path) => {
                        println!("{}", jade_swarm.write_results(path))
                    }
                    None => {
                        println!("No DFS solutions were found.")
                    }
                }
            },
            SearchMethod::DFSWithCycleChecking(true) => {
                println!("DFS With Cycle Checking: ");
                match jade_swarm.depth_first_search_with_cycle_checking() {
                    Some(path) => {
                        println!("{}", jade_swarm.write_results(path))
                    }
                    None => {
                        println!("No DFS solutions were found.")
                    }
                }
            },
            SearchMethod::DFSWithoutCycleChecking(true) => {
                println!("DFS Without Cycle Checking: ");
                match jade_swarm.depth_first_search_without_cycle_checking() {
                    Some(path) => {
                        println!("{}", jade_swarm.write_results(path))
                    }
                    None => {
                        println!("No DFS solutions were found.")
                    }
                }
            },
            SearchMethod::IDSReached(true) => {
                println!("IDS With Reached: ");
                match jade_swarm.iterative_deepening_search_with_reached() {
                    Some(path) => {
                        println!("{}", jade_swarm.write_results(path))
                    }
                    None => {
                        println!("No IDA solutions were found.")
                    }
                }
            },
            SearchMethod::IDSWithoutCycleChecking(true) => {
                println!("IDS Without Cycle Checking: ");
                match jade_swarm.iterative_deepening_search_without_cycle_checking() {
                    Some(path) => {
                        println!("{}", jade_swarm.write_results(path))
                    }
                    None => {
                        println!("No IDA solutions were found.")
                    }
                }
            },
            SearchMethod::IDSWithCycleChecking(true) => {
                println!("IDS With Cycle Checking: ");
                match jade_swarm.iterative_deepening_search_with_cycle_checking() {
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
                println!("RBFS: ");
                match jade_swarm.recursive_best_first_search() {
                    Some(path) => {
                        println!("{}", jade_swarm.write_results(path))
                    }
                    None => {
                        println!("No RBFS solutions were found.")
                    }
                }
            },
            SearchMethod::IDAStarReached(true) => {
                println!("IDA* With Reached: ");
                match jade_swarm.iterative_deepening_a_star_with_reached() {
                    Some(path) => {
                        println!("{}", jade_swarm.write_results(path))
                    }
                    None => {
                        println!("No IDA* solutions were found.")
                    }
                }
            },
            SearchMethod::IDAStarWithCycleChecking(true) => {
                println!("IDA* With Cycle Checking: ");
                match jade_swarm.iterative_deepening_a_star_with_cycle_checking() {
                    Some(path) => {
                        println!("{}", jade_swarm.write_results(path))
                    }
                    None => {
                        println!("No IDA* solutions were found.")
                    }
                }
            },
            SearchMethod::IDAStarWithoutCycleChecking(true) => {
                println!("IDA* Without Cycle Checking: ");
                match jade_swarm.iterative_deepening_a_star_without_cycle_checking() {
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
