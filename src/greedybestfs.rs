use crate::problem::JadeSwarm;
use crate::problem::Node;

impl JadeSwarm {
/*
* --------------------------------------
* --------------------------------------
*  Greedy Best First Search
* --------------------------------------
* --------------------------------------
*/
    pub fn greedy_best_first_search(&self) -> Option<Vec<Node>> {
        let g = |_node: &Node| -> i32 { 0 };

        // Consistent and admissible heuristic:
        /*
         * Negative of ((number of non-green nodes) / (maximum branching factor of graph))
         */
        let maximum_branching_factor = self.maximum_branching_factor;

        let h = |node: &Node| -> i32 {
            let x = f64::from(
                node.state.config.len() as i32
                    - node.state.config.iter().filter(|&b| (*b) == true).count() as i32,
            ) / maximum_branching_factor as f64;
            x.ceil() as i32
        };

        self.best_first_search(g, h)
    }    
}
