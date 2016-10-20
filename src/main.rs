extern crate piston_window;
extern crate graphics;
extern crate find_folder;
extern crate deeplearning;
extern crate nalgebra;
extern crate ncollide;
extern crate json_logger;
extern crate log;

use log::LogLevelFilter;
use piston_window::*;

pub mod graphics_app;
pub mod intelligence;
pub mod simulation;
pub mod map;

use graphics_app::App;
use simulation::Simulation;
use intelligence::Intelligence;
use map::Map;

fn main () {
    json_logger::init("sdc", LogLevelFilter::Debug).unwrap();

    let opengl = OpenGL::V3_3;

    // Load font
    let assets = find_folder::Search::ParentsThenKids(3, 3)
        .for_folder("assets").unwrap();

    // Create Simulation
    let map = Map::demo();
    let map2 = Map::demo2();
    let mut simulation = Simulation::new(&map, map.build_collision_shape());
    let mut second = simulation.clone();

    let mut intelligence = Intelligence::new(&mut simulation);
    //intelligence.train(2);

    match Intelligence::train_scaled(&mut second) {
        Some(intel) => {
            intelligence = intel;
        },

        None => {
            return;
        }
    };

    // Create a new game and run it.
    let mut app = App::new(&map, intelligence.simulation.clone());

    // Create an Glutin window.
    let mut window: PistonWindow = WindowSettings::new("self-driving-car - 2d simulation", [1024, 760])
        .opengl(opengl)
        .exit_on_esc(true)
        .build()
        .unwrap();

    let ref font = assets.join("FiraSans-Regular.ttf");
    let factory = window.factory.borrow().clone();
    let mut glyphs = Glyphs::new(font, factory).unwrap();

    window.set_max_fps(60);

    let mut tick = 0;
    // Render loop
    for e in window {
        tick += 1;

        // run logic
        intelligence.tick();

        // give renderer data from current simulation state
        app.simulation = intelligence.simulation.clone();

        if let Some(args) = e.render_args() {
        	app.render(&mut glyphs, &mut &e, &args);
        }

        if let Some(button) = e.press_args() {
            match button {
                Button::Keyboard(key) => {
                    match key {
                        Key::R => {
                            intelligence.simulation.reset(&map);
                            println!("Replay :)");
                        },
                        Key::T => {
                            println!("Continue training..");
                            intelligence.simulation.reset(&map);
                            intelligence.simulation.update_colission(map.build_collision_shape());
                            intelligence.continue_train(5);
                            intelligence.simulation.reset(&map);
                        },
                        Key::D1 => {
                            intelligence.simulation.reset(&map);
                            intelligence.simulation.update_colission(map.build_collision_shape());
                            app.map = &map;
                        },
                        Key::D2 => {
                            intelligence.simulation.reset(&map2);
                            intelligence.simulation.update_colission(map2.build_collision_shape());
                            app.map = &map2;
                        },
                        _ => {}
                    }
                },
                _ => {}
            }
        }
    }

}
