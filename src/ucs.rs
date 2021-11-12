use crate::problem::JadeSwarm;
use crate::problem::Node;
use crate::problem::StoneColor;

impl JadeSwarm {
/*
* --------------------------------------
* --------------------------------------
*  Uniform Cost Search
* --------------------------------------
* --------------------------------------
*/
    pub fn uniform_cost_search(&self) -> Option<Vec<Node>> {
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
}
