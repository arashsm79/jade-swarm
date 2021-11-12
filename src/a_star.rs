use crate::problem::JadeSwarm;
use crate::problem::Node;

impl JadeSwarm {
/*
* --------------------------------------
* --------------------------------------
*  A* Search
* --------------------------------------
* --------------------------------------
*/
    pub fn a_star(&self) -> Option<Vec<Node>> {
        let g = |_node: &Node| -> i32 { 1 };

        // Consistent and admissible heuristic:
        /*
         * ((number of non-green nodes) / (maximum branching factor of graph))
         */
        let maximum_branching_factor = self.maximum_branching_factor + 1;

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
