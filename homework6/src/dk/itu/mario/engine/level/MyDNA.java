package dk.itu.mario.engine.level;

import java.util.Random;
import java.util.*;

//Make any new member variables and functions you deem necessary.
//Make new constructors if necessary
//You must implement mutate() and crossover()


public class MyDNA extends DNA {

    public int numGenes = 0; //number of genes

    // Use these constants to make your DNA strings.

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
    private static final char[] BLOCK_TYPES = {GAP_CHAR, STRAIGHT_CHAR, COINS_CHAR, HILL_CHAR, MONSTERS_CHAR};

    // Return a new DNA that differs from this one in a small way.
    // Do not change this DNA by side effect; copy it, change the copy, and return the copy.
    public MyDNA mutate() {
        MyDNA copy = new MyDNA();
        StringBuilder chromosomeBuilder = new StringBuilder(this.getChromosome());
        Random rand = new Random();
        int num_gene_mutated = rand.nextInt(10);
        //YOUR CODE GOES BELOW HERE
        int indx_mutated;
        int rand_block;
        int rand_dup;

        for (int i = 0; i < num_gene_mutated; i++) { // mutation
            indx_mutated = rand.nextInt(this.getChromosome().length());
            rand_block = rand.nextInt(BLOCK_TYPES.length);
            rand_dup = rand.nextInt(10);
            if (indx_mutated % 2 == 0) { // even indx: block
                chromosomeBuilder.setCharAt(indx_mutated + 1, BLOCK_TYPES[rand_block]);
                chromosomeBuilder.setCharAt(indx_mutated, (char) rand_dup);
            } else { // odd: duplicate block number
                chromosomeBuilder.setCharAt(indx_mutated - 1, BLOCK_TYPES[rand_block]);
                chromosomeBuilder.setCharAt(indx_mutated, (char) rand_dup);
            }
        }

        int strip_dna = rand.nextInt(3);

        if (strip_dna == 2) { // remove first piece of gene
            chromosomeBuilder.substring(2);
        } else if (strip_dna == 1) { // add one more piece of gene
            char block = BLOCK_TYPES[rand.nextInt(BLOCK_TYPES.length)];
            chromosomeBuilder.append(block).append(rand.nextInt(10));
        } else { // do nothing
        }

        copy.setChromosome(chromosomeBuilder.toString());
        //YOUR CODE GOES ABOVE HERE
        return copy;
    }

    // Do not change this DNA by side effect
    public ArrayList<MyDNA> crossover(MyDNA mate) {
        ArrayList<MyDNA> offsprings = new ArrayList<MyDNA>();
        StringBuilder offspringChromosomeBuilder = new StringBuilder(this.getChromosome());
        StringBuilder mateChromosomeBuilder = new StringBuilder(mate.getChromosome());
        MyDNA shorter = this.getChromosome().length() < mate.getChromosome().length() ? this : mate;

        //YOUR CODE GOES BELOW HERE
        Random rand = new Random();
        int numOffspring = rand.nextInt(10) + 1;
        for (int i = 0; i < numOffspring; i++) {
            int shorterMid = shorter.getChromosome().length() / 2;
            int startCross = rand.nextInt(shorterMid);
            int endCross = rand.nextInt(shorterMid - 1) + shorterMid;
            offspringChromosomeBuilder.replace(startCross, endCross, mateChromosomeBuilder.substring(startCross, endCross));
            MyDNA newOffspring = new MyDNA();
            newOffspring.setChromosome(offspringChromosomeBuilder.toString());
            offsprings.add(newOffspring);
        }
        //YOUR CODE GOES ABOVE HERE
        return offsprings;
    }

    // Optional, modify this function if you use a means of calculating fitness other than using the fitness member variable.
    // Return 0 if this object has the same fitness as other.
    // Return -1 if this object has lower fitness than other.
    // Return +1 if this objet has greater fitness than other.
    public int compareTo(MyDNA other) {
        int result = super.compareTo(other);
        //YOUR CODE GOES BELOW HERE

        //YOUR CODE GOES ABOVE HERE
        return result;
    }

    public boolean equals(MyDNA other) {
        return this.getChromosome().equals(other.getChromosome()) &&
                this.getFitness() == other.getFitness();
    }


    // For debugging purposes (optional)
    public String toString() {
        String s = super.toString();
        //YOUR CODE GOES BELOW HERE

        //YOUR CODE GOES ABOVE HERE
        return s;
    }

    public void setNumGenes(int n) {
        this.numGenes = n;
    }

}

