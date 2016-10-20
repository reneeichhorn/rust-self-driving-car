use deeplearning::*;

use simulation::Simulation;

pub struct Intelligence<'a> {
    pub simulation: &'a mut Simulation,
    pub network: Box<NeuralNetwork>,
}

impl<'a> Intelligence<'a> {
    pub fn new (sim: &'a mut Simulation) -> Self {
        // build network
        let mut network: Box<NeuralNetwork> = Box::new(NeuralNetwork::new());
        network.set_inputs(2 + sim.sensors.len());
        network.add_neuron_group(0, NeuronType::TanH, 6, -1.0, 1.0);
        network.add_neuron_group(1, NeuronType::TanH, 2, -1.0, 1.0);
        network.build();

        let intel = Intelligence {
            simulation: sim,
            network: network,
        };

        return intel;
    }

    pub fn train_scaled(sim: &mut Simulation) -> Option<Intelligence> {

        let options = Options::defaults();

        match process_type() {
            ProcessType::Master => {
                for _ in 0..6 {
                    fork_scaling();
                }

                let result = create_scaling_host::<Box<NeuralNetwork>>(options);
                match result {
                    Some(obj) => {
                        Some(Intelligence {
                            simulation: sim,
                            network: obj
                        })
                    },
                    None => {
                        None
                    }
                }
            },
            ProcessType::Slave => {
                create_scaling_client(options.port, || -> (Box<NeuralNetwork>, f64) {
                    let mut network: NeuralNetwork = NeuralNetwork::new();
                    network.set_inputs(2 + sim.sensors.len());
                    network.add_neuron_group(0, NeuronType::TanH, 2 + sim.sensors.len(), -1.0, 1.0);
                    network.add_neuron_group(1, NeuronType::TanH, 2, -1.0, 1.0);
                    network.build();

                    let mut intel = Intelligence {
                        simulation: &mut sim.clone(),
                        network: Box::new(network)
                    };
                    let fitness = intel.train(5);
                    (intel.network, fitness)
                }, | &mut ref network | -> f64 {
                    let mut intel = Intelligence {
                        simulation: &mut sim.clone(),
                        network: network.clone()
                    };
                    intel.continue_train(5)
                });

                None
            }
        }
    }

    pub fn train (&mut self, generations: usize) -> f64 {
        let (net, fitness) = genetic_evolution(25, StopRule::GenerationReached(generations), &mut | index | {
            let mut network: NeuralNetwork = NeuralNetwork::new();
            network.set_inputs(2 + self.simulation.sensors.len());
            network.add_neuron_group(0, NeuronType::TanH, 2 + self.simulation.sensors.len(), -1.0, 1.0);
            network.add_neuron_group(1, NeuronType::TanH, 2, -1.0, 1.0);
            network.build();
            network
        }, &mut | network | {
            let mut sim = self.simulation.clone();

            let mut score = 0.0;
            for tick in 0..100 {
                Self::tick_over(network, &mut sim, 0.015);
                score += sim.local_velocity.x;
            };
            score
        }, None);

        self.network = Box::new(net);
        fitness
    }

    pub fn continue_train (&mut self, generations: usize) -> f64 {
        let (net, fitness) = genetic_evolution(25, StopRule::GenerationReached(generations), &mut | index | {
            let mut clone = *self.network.clone();
            clone.mutate();
            clone
        }, &mut | network | {
            let mut sim = self.simulation.clone();

            let mut score = 0.0;
            for tick in 0..100 {
                Self::tick_over(network, &mut sim, 0.015);
                score += sim.local_velocity.x;
            };
            score
        }, None);

        self.network = Box::new(net);
        fitness
    }

    pub fn tick_over(net: &NeuralNetwork, sim: &mut Simulation, elapsed: f64) {
        // create processing unit
        let mut instance = CpuInstance::new(net).unwrap();

        // create inputs
        let mut inputs = vec![
            sim.headed,
            sim.local_velocity.x / 140.0,
        ];

        for sensor in &sim.sensors {
            inputs.push(sensor.range / sensor.max_range);
        }

        // calculate outputs and redirect them to the simulation
        let mut outputs: Vec<f64> = Vec::new();
        instance.calculate(&inputs, &mut outputs);

        sim.set_acceleration_input(outputs[0]);
        sim.steering = outputs[1];

        // update simulation
        sim.tick(elapsed);

    }

    pub fn tick (&mut self) {
        Self::tick_over(self.network.as_ref(), self.simulation, 0.01);
    }
}
