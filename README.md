<h1>Rock Paper Scissors Game<h1>


The main purpose of this project is to detect hand symbols of between two players simultaneously from a video feed and update the score accordingly. The system was implemented using an OMAP L138 Development Kit and the vpif_lcd_loopback framework provided in the C6748 LCDK Starterware library. The video feed of the hand signals was processed to facilitate the extraction of feature vectors. The features were then used to train a single-hidden-layer neural net, whose coefficient would later allow for fast real-time detection of the hand symbols. With RGB color code model and Breadth First Search algorithm this process was executed with speedy performance.
