package dk.itu.mario.engine.level.generator;

import java.util.*;

import com.sun.deploy.util.ArrayUtil;
import dk.itu.mario.MarioInterface.Constraints;
import dk.itu.mario.MarioInterface.GamePlay;
import dk.itu.mario.MarioInterface.LevelGenerator;
import dk.itu.mario.MarioInterface.LevelInterface;
import dk.itu.mario.engine.level.Level;
import dk.itu.mario.engine.level.MyLevel;
import dk.itu.mario.engine.level.MyDNA;

import dk.itu.mario.engine.PlayerProfile;

import dk.itu.mario.engine.sprites.SpriteTemplate;
import dk.itu.mario.engine.sprites.Enemy;

public class MyLevelGenerator {

    public boolean verbose = true; //print debugging info
    private final static int POP_SIZE = 300;
    private final static int CROSS_TIMES = 100;

    // Represents a gap in the floor that Mario can fall through and die.
    public static final char GAP_CHAR = 'G';
    // Represents a straight, flat section of ground.
    public static final char STRAIGHT_CHAR = 'S';
    // Represents ground with coins above it.
    public static final char COINS_CHAR = 'C';
    // Represents a set of stairs that Mario needs to jump over.
    public static final char HILL_CHAR = 'H';
    // Represents ground with monsters over it (e.g., goombas, koopas).
    public static final char MONSTERS_CHAR = 'M';

    // MAKE ANY NEW MEMBER VARIABLES HERE

    // Called by the game engine.
    // Returns the level to be played.
    public Level generateLevel(PlayerProfile playerProfile) {
        // Call genetic algorithm to optimize to the player profile
        MyDNA dna = this.geneticAlgorithm(playerProfile);

        // Post process
        dna = this.postProcess(dna);

        // Convert the solution to the GA into a Level
        MyLevel level = new MyLevel(dna, LevelInterface.TYPE_OVERGROUND);

        if (this.verbose) {
            System.out.println("Solution: " + dna + " fitness: " + playerProfile.evaluateLevel(level));
        }

        return (Level) level;
    }

    private double maxFitness(Iterable<MyDNA> pop) {
        double max = -1;
        for (MyDNA eachDNA : pop) {
            if (eachDNA.getFitness() > max) {
                max = eachDNA.getFitness();
            }
        }
        return max;
    }

    // Genetic Algorithm implementation
    private MyDNA geneticAlgorithm(PlayerProfile playerProfile) {
        // Set the population size
        int populationSize = getPopulationSize();

        // Make the population array
        ArrayList<MyDNA> population = new ArrayList<MyDNA>();

        // Make the solution, which is initially null
        MyDNA solution = null;

        // Generate a random population
        for (int i = 0; i < populationSize; i++) {
            MyDNA newIndividual = this.generateRandomIndividual();
            newIndividual.setFitness(this.evaluateFitness(newIndividual, playerProfile));
            population.add(newIndividual);
        }
        if (this.verbose) {
            System.out.println("Initial population:");
            printPopulation(population);
        }

        // Iteration counter
        int count = 0;

        // Iterate until termination criteria met
        while (!this.terminate(population, count)) {
            // Make a new, possibly larger population
            ArrayList<MyDNA> newPopulation = new ArrayList<MyDNA>();

            // Keep track of individual's parents (for this iteration only)
            Hashtable<MyDNA, ArrayList<MyDNA>> parents = new Hashtable<>();

            // Mutuate a number of individuals
            ArrayList<MyDNA> mutationPool = this.selectIndividualsForMutation(population);
            for (int i = 0; i < mutationPool.size(); i++) {
                MyDNA parent = mutationPool.get(i);
                // Mutate
                MyDNA mutant = parent.mutate();
                // Evaluate fitness
                double fitness = this.evaluateFitness(mutant, playerProfile);
                mutant.setFitness(fitness);
                // Add mutant to new population
                newPopulation.add(mutant);
                // Create a list of parents and remember it in a hash
                ArrayList<MyDNA> p = new ArrayList<MyDNA>();
                p.add(parent);
                parents.put(mutant, p);
            }


            // Do Crossovers
            for (int i = 0; i < this.numberOfCrossovers(); i++) {
                // Pick two parents
                MyDNA parent1 = this.pickIndividualForCrossover(newPopulation, null);
                MyDNA parent2 = this.pickIndividualForCrossover(newPopulation, parent1);

                if (parent1 != null && parent2 != null) {
                    // Crossover produces one or more children
                    ArrayList<MyDNA> children = parent1.crossover(parent2);

                    // Add children to new population and remember their parents
                    for (int j = 0; j < children.size(); j++) {
                        // Get a child
                        MyDNA child = children.get(j);
                        // Evaluate fitness
                        double fitness = this.evaluateFitness(child, playerProfile);
                        child.setFitness(fitness);
                        // Add it to new population
                        newPopulation.add(child);
                        // Create a list of parents and remember it in a hash
                        ArrayList<MyDNA> p = new ArrayList<MyDNA>();
                        p.add(parent1);
                        p.add(parent2);
                        parents.put(child, p);
                    }
                }

            }

            // Cull the population
            // There is more than one way to do it.
            if (this.competeWithParentsOnly()) {
                population = this.competeWithParents(population, newPopulation, parents);
            } else {
                population = this.globalCompetition(population, newPopulation);
            }

            //increment counter
            count = count + 1;

            if (this.verbose) {
                MyDNA best = this.getBestIndividual(population);
                System.out.println("" + count + ": Best: " + best + " fitness: " + best.getFitness());
            }
        }

        // Get the winner
        solution = this.getBestIndividual(population);

        return solution;
    }

    // Create a random individual.
    private MyDNA generateRandomIndividual() {
        MyDNA individual = new MyDNA();
        // YOUR CODE GOES BELOW HERE
        char[] block_types = {GAP_CHAR, STRAIGHT_CHAR, COINS_CHAR, HILL_CHAR, MONSTERS_CHAR};
        Random rand = new Random();
        int rand_size = rand.nextInt(50) + 50;
        StringBuilder chromosome = new StringBuilder();
        int rand_indx;
        int rand_dup;
        while (chromosome.length() < rand_size) {
            rand_indx = rand.nextInt(block_types.length);
            rand_dup = rand.nextInt(9) + 1;
            chromosome.append(block_types[rand_indx]).append(Integer.toString(rand_dup));
        }
        individual.setChromosome(chromosome.toString());

        // YOUR CODE GOES ABOVE HERE
        return individual;
    }

    // Returns true if the genetic algorithm should terminate.
    private boolean terminate(ArrayList<MyDNA> population, int count) {
        boolean decision = false;
        // YOUR CODE GOES BELOW HERE
        for (MyDNA eachDNA : population) {
            if (eachDNA.getFitness() > 0.8) {
                decision = true;
                break;
            }
        }
        System.out.println("Max Fitness: " + maxFitness(population));
        // YOUR CODE GOES ABOVE HERE
        return decision;
    }

    // Return a list of individuals that should be copied and mutated.
    private ArrayList<MyDNA> selectIndividualsForMutation(ArrayList<MyDNA> population) {
        ArrayList<MyDNA> selected = new ArrayList<MyDNA>();
        // YOUR CODE GOES BELOW HERE
        Random rand = new Random();
        int selected_size = rand.nextInt(population.size() / 2);
        int selected_indx = rand.nextInt(population.size());
        for (int i = 0; i < selected_size; i++) {
            selected.add(population.get(selected_indx));
        }
        // YOUR CODE GOES ABOVE HERE
        return selected;
    }

    // Returns the size of the population.
    private int getPopulationSize() {
        int num = POP_SIZE; // Default needs to be changed
        // YOUR CODE GOES BELOW HERE

        // YOUR CODE GOES ABOVE HERE
        return num;
    }

    // Returns the number of times crossover should happen per iteration.
    private int numberOfCrossovers() {
        int num = CROSS_TIMES; // Default is no crossovers
        // YOUR CODE GOES BELOW HERE

        // YOUR CODE GOES ABOVE HERE
        return num;

    }

    // Pick one of the members of the population that is not the same as excludeMe
    private MyDNA pickIndividualForCrossover(ArrayList<MyDNA> population, MyDNA excludeMe) {
        MyDNA picked = null;
        Random rand = new Random();
        if (excludeMe == null) {
            return population.get(rand.nextInt(population.size()));
        }
        // YOUR CODE GOES BELOW HERE
        int selected_indx = 0;
        while (picked == null || picked.equals(excludeMe)) {
            selected_indx = rand.nextInt(population.size());
            picked = population.get(selected_indx);
        }
        // YOUR CODE GOES ABOVE HERE
        if (picked == excludeMe) {
            return null;
        } else {
            return picked;
        }
    }

    // Returns true if children compete to replace parents.
    // Retursn false if the the global population competes.
    private boolean competeWithParentsOnly() { // change it to test
        boolean doit = false;
        // YOUR CODE GOES BELOW HERE

        // YOUR CODE GOES ABOVE HERE
        return doit;
    }

    // Determine if children are fitter than parents and keep the fitter ones.
    private ArrayList<MyDNA> competeWithParents(ArrayList<MyDNA> oldPopulation,
                                                ArrayList<MyDNA> newPopulation,
                                                Hashtable<MyDNA, ArrayList<MyDNA>> parents) {
        ArrayList<MyDNA> finalPopulation = new ArrayList<MyDNA>();
        // YOUR CODE GOES BELOW HERE
        for (MyDNA child : parents.keySet()) {
            for (MyDNA parent : parents.get(child)) {
                if (child.getFitness() > parent.getFitness() && oldPopulation.indexOf(parent) >= 0) {
                    oldPopulation.remove(parent);
                    break;
                }
            }
        }

        ArrayList<MyDNA> buffer = new ArrayList<>();
        buffer.addAll(oldPopulation);
        buffer.addAll(newPopulation);
        Collections.sort(buffer);

        for (int i = buffer.size() - 1; i >= 0; i--) {
            if (buffer.size() < this.getPopulationSize()) {
                finalPopulation.add(buffer.get(i));
            }
        }

        // YOUR CODE GOES ABOVE HERE
        if (finalPopulation.size() != this.getPopulationSize()) {
            throw new IllegalStateException("Population not the correct size.");
        }
        return finalPopulation;
    }

    // Combine the old population and the new population and return the top fittest individuals.
    private ArrayList<MyDNA> globalCompetition(ArrayList<MyDNA> oldPopulation, ArrayList<MyDNA> newPopulation) {
        ArrayList<MyDNA> finalPopulation = new ArrayList<MyDNA>();
        // YOUR CODE GOES BELOW HERE
        ArrayList<MyDNA> buffer = new ArrayList<>();
        buffer.addAll(oldPopulation);
        buffer.addAll(newPopulation);
        Collections.sort(buffer);
        for (int i = buffer.size() - 1; i >= 0; i--) {
            if (finalPopulation.size() < this.getPopulationSize()) {
                finalPopulation.add(buffer.get(i));
            }
        }
//        System.out.println("Pop size: " + finalPopulation.size());
        // YOUR CODE GOES ABOVE HERE
        if (finalPopulation.size() != this.getPopulationSize()) {
            throw new IllegalStateException("Population not the correct size.");
        }
        return finalPopulation;
    }

    // Return the fittest individual in the population.
    private MyDNA getBestIndividual(ArrayList<MyDNA> population) {
        MyDNA best = population.get(0);
        double bestFitness = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < population.size(); i++) {
            MyDNA current = population.get(i);
            double currentFitness = current.getFitness();
            if (currentFitness > bestFitness) {
                best = current;
                bestFitness = currentFitness;
            }
        }
        return best;
    }

    // Changing this function is optional.
    private double evaluateFitness(MyDNA dna, PlayerProfile playerProfile) {
        double fitness = 0.0;
        // YOUR CODE GOES BELOW HERE
        MyLevel level = new MyLevel(dna, LevelInterface.TYPE_OVERGROUND);
        fitness = playerProfile.evaluateLevel(level);
        // YOUR CODE GOES ABOVE HERE
        return fitness;
    }

    private MyDNA postProcess(MyDNA dna) {
        // YOUR CODE GOES BELOW HERE

        // YOUR CODE GOES ABOVE HERE
        return dna;
    }

    //for this to work, you must implement MyDNA.toString()
    private void printPopulation(ArrayList<MyDNA> population) {
        for (int i = 0; i < population.size(); i++) {
            MyDNA dna = population.get(i);
            System.out.println("Individual " + i + ": " + dna + " fitness: " + dna.getFitness());
        }
    }

    // MAKE ANY NEW MEMBER FUNCTIONS HERE

}
