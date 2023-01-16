/*
 * Title:        CloudSim Toolkit
 * Description:  CloudSim (Cloud Simulation) Toolkit for Modeling and Simulation
 *               of Clouds
 * Licence:      GPL - http://www.gnu.org/copyleft/gpl.html
 *
 * Copyright (c) 2009, The University of Melbourne, Australia
 */

package org.cloudbus.cloudsim.examples;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Arrays;

import org.cloudbus.cloudsim.Cloudlet;
import org.cloudbus.cloudsim.CloudletSchedulerTimeShared;
import org.cloudbus.cloudsim.Datacenter;
import org.cloudbus.cloudsim.DatacenterBroker;
import org.cloudbus.cloudsim.DatacenterCharacteristics;
import org.cloudbus.cloudsim.Host;
import org.cloudbus.cloudsim.Log;
import org.cloudbus.cloudsim.Pe;
import org.cloudbus.cloudsim.Storage;
import org.cloudbus.cloudsim.UtilizationModel;
import org.cloudbus.cloudsim.UtilizationModelFull;
import org.cloudbus.cloudsim.Vm;
import java.util.Random;
import org.cloudbus.cloudsim.VmAllocationPolicy;
import org.cloudbus.cloudsim.VmSchedulerTimeShared;
import org.cloudbus.cloudsim.core.CloudSim;
import org.cloudbus.cloudsim.provisioners.BwProvisionerSimple;
import org.cloudbus.cloudsim.provisioners.PeProvisionerSimple;
import org.cloudbus.cloudsim.provisioners.RamProvisionerSimple;

/**
 * An example showing how to create
 * scalable simulations.
 */
public class VmAllocationPolicyBat extends VmAllocationPolicy {

	private double[][] POP_SOL; 		// Population/Solution (BATS x D) 
	private double[][] V; 		// Velocities (BATS x D)
	private double[][] FRE; 		// Frequency : 0 to F_MAX (BATS x 1)
	private double[] FIT;			// Fitness (BATS)
	private double PR; 			// Pulse Rate : 0 to 1
	private double L; 			// Louadness : L_MIN to L_MAX
	private double[][] lb;		// Lower bound (1 x D)
	private double[][] ub;		// Upper bound (1 x D)
	private double fmin; 		// Minimum fitness from FIT
 	private double[] B;			// Best solution array from POP_SOL (D)	

	private  int BATS; 		// Number of bats
	private  int MAX; 		// Number of iterations
	private  double F_MIN = 0.0;
	private  double F_MAX = 2.0;
	private  double L_MIN;
	private  double L_MAX;
	private  double PR_MIN;
	private  double PR_MAX; 
	private  int D = 10;
	private List<Host> hosts;
	private  Random rand = new Random();


	public void BatAlgo(List<Host> hosts, int MAX, double L_MIN, double L_MAX, double PR_MIN, double PR_MAX){
		this.hosts = hosts;
		this.BATS = hosts.size();
		this.MAX = MAX;
		this.PR_MAX = PR_MAX;
		this.PR_MIN = PR_MIN;
		this.L_MAX = L_MAX;
		this.L_MIN = L_MIN;

		this.POP_SOL = new double[BATS][D];
		this.V = new double[BATS][D];
		this.FRE = new double[BATS][1];
		this.FIT = new double[BATS];
		this.PR = (PR_MAX + PR_MIN) / 2;
		this.L = (L_MIN + L_MAX) / 2;

		// Initialize bounds
		this.lb = new double[1][D];
		for ( int i = 0; i < D; i++ ){
			this.lb[0][i] = -2.0;
		}
		this.ub = new double[1][D];
		for ( int i = 0; i < D; i++ ){
			this.ub[0][i] = 2.0;
		}

		// Initialize FRE and V
		for ( int i = 0; i < BATS; i++ ){
			this.FRE[i][0] = 0.0;
		}
		for ( int i = 0; i < BATS; i++ ){
			for ( int j = 0; j < D; j++ ) {
				this.V[i][j] = 0.0;
			}
		}

		// Initialize POP_SOL
		for ( int i = 0; i < BATS; i++ ){
			for ( int j = 0; j < D; j++ ){
				// this.POP_SOL[i][j] = lb[0][j] + (ub[0][j] - lb[0][j]) * rand.nextDouble();
				// For SLA, power Consumption, RRR 
				this.POP_SOL[i][j] = 0.1 * rand.nextDouble() * 3; 
			}
			this.FIT[i] = objective(POP_SOL[i], i);
		}

		// Find initial best solution
		int fmin_i = 0;
		for ( int i = 0; i < BATS; i++ ){
			if ( FIT[i] < FIT[fmin_i] )
				fmin_i = i;
		}

		// Store minimum fitness and it's index.
		// B holds the best solution array[1xD]
		this.fmin = FIT[fmin_i];
		this.B = POP_SOL[fmin_i]; // (1xD)
	}

    // Can be replaced with ROSENBROCK 3d FUNCTION (https://www.sfu.ca/~ssurjano/rosen.html) easy valley but difficult global minima
	private double objective(double[] Xi, int index){
		double sum = 0.0;
		Host temp = hosts.get(index);
		// temp.getAvailableMips();
		// temp.getRamProvisioner().getAvailableRam()
		
		for ( int i = 0; i < Xi.length; i++ ){
			sum = sum + Xi[i] * Xi[i];
		}
		return sum;
	}

	private double[] simpleBounds(double[] Xi){
		// Don't know if this should be implemented
		double[] Xi_temp = new double[D];
		System.arraycopy(Xi, 0, Xi_temp, 0, D);

		for ( int i = 0; i < D; i++ ){
			if ( Xi_temp[i] < lb[0][i] )
				Xi_temp[i] = lb[0][i];
			else continue;
		}

		for ( int i = 0; i < D; i++ ){
			if ( Xi_temp[i] > ub[0][i] )
				Xi_temp[i] = lb[0][i];
			else continue;
		}
		return Xi_temp;
	}



	/** The map between each VM and its allocated host.
         * The map key is a VM UID and the value is the allocated host for that VM. */
	private Map<String, Host> vmTable;

	/** The map between each VM and the number of Pes used. 
         * The map key is a VM UID and the value is the number of used Pes for that VM. */
	private Map<String, Integer> usedPes;

	/** The number of free Pes for each host from {@link #getHostList() }. */
	private List<Integer> freePes;

	public VmAllocationPolicyBat(List<? extends Host> list) {
		super(list);

		setFreePes(new ArrayList<Integer>());
		for (Host host : getHostList()) {
			getFreePes().add(host.getNumberOfPes());

		}

		setVmTable(new HashMap<String, Host>());
		setUsedPes(new HashMap<String, Integer>());
	}
	@Override
	public boolean allocateHostForVm(Vm vm) {
		int requiredPes = vm.getNumberOfPes();
		boolean result = false;
		int tries = 0;
		List<Integer> freePesTmp = new ArrayList<Integer>();
		for (Integer freePes : getFreePes()) {
			freePesTmp.add(freePes);
		}

		if (!getVmTable().containsKey(vm.getUid())) { // if this vm was not created
			do {// we still trying until we find a host or until we try all of them
				int moreFree = Integer.MIN_VALUE;
				int idx = -1;

				this.BatAlgo(getHostList(), 1000, 0.0, 1.0, 0.0, 1.0);
				idx  = this.startBat();
				// Original Code
				// // we want the host with less pes in use
				// for (int i = 0; i < freePesTmp.size(); i++) {
				// 	if (freePesTmp.get(i) > moreFree) {
				// 		moreFree = freePesTmp.get(i);
				// 		idx = i;
				// 	}
				// }

				Host host = getHostList().get(idx);
				result = host.vmCreate(vm);

				if (result) { // if vm were succesfully created in the host
					getVmTable().put(vm.getUid(), host);
					getUsedPes().put(vm.getUid(), requiredPes);
					getFreePes().set(idx, getFreePes().get(idx) - requiredPes);
					result = true;
					break;
				} else {
					freePesTmp.set(idx, Integer.MIN_VALUE);
				}
				tries++;
			} while (!result && tries < getFreePes().size());

		}

		return result;
	}


	public int  startBat(){

		double[][] S = new double[BATS][D];
		int n_iter = 0;
		int indexFinal = -1;
        // int node = BATS;
        // double[] nodes = new double[node];
        // Calculate and fill getResourceRemainingRate for all CPUS here
        // double[] rrR = new double[node];
        // Calculate and fill getPowerConsumption for all CPUS here
        // double[] pC = new double[node];
        // Calculate and fill getSla for all CPUS here
        // double[] sLA = new double[node];
        // for (int i = 0; i < node; i++) {
        //     rrR[i] = getResourceRemainingRate(nodes[i].cpu_utilized,nodes[i].cpu_total,nodes[i].mem_utilized, nodes[i].mem_total, nodes[i].bw_utilized, nodes[i].bw_total)
        //     pC[i] = getPowerConsumption(nodes[i].cpu_utilized,nodes[i].cpu_total, nodes[i].mem_utilized, nodes[i].mem_total, nodes[i].bw_utilized, nodes[i].bw_total)
        //     sLA[i] = getSla(nodes[i].cpu_utilized,nodes[i].cpu_total)
        // }
        // Arrays.sort(rrR);
        // F_MIN = rrR[0];
        // F_MAX = rrR[rrR.length-1];

        
        // for (int i = 0; i < node; i++) {
        // }
		
        // Loop for all iterations/generations(MAX)
		for ( int t = 0; t < MAX; t++ ){
			// Loop for all bats(BATS) or NODES--
			for ( int i = 0; i < BATS; i++ ){
				
				// Update frequency (Nx1)
				FRE[i][0] = F_MIN + (F_MIN-F_MAX) * (rand.nextDouble() ); // Fmin and fmax will be replaced by node range of getResourceRemainingRate for all cpus
				// Update velocity (NxD)
				for ( int j = 0; j < D; j++ ){
					V[i][j] = V[i][j] + (POP_SOL[i][j] - B[j]) * FRE[i][0];
				}
				// Update S = POP_SOL + V
				for ( int j = 0; j < D; j++ ){
					S[i][j] = POP_SOL[i][j] + V[i][j];
				}
				// Apply bounds/limits
				S[i] = simpleBounds(POP_SOL[i]);
				

                // PR will be replaced by getPowerConsumption(cpu_utilized, cpu_total, p_min, p_max) with a very gradual increase

                // Pulse rate
				if ( rand.nextDouble() > PR ) //pC
					for ( int j = 0; j < D; j++ )
                    {
                        // Generating Local solution around best solution
						// 0.001 is the learning rate
						S[i][j] = B[j] + 0.001 * rand.nextGaussian(); // distribution of random data from -3 to 3 https://www.javamex.com/tutorials/random_numbers/gaussian_distribution_2.shtml 
                    }

				// Evaluate new solutions
				double fnew = objective(S[i], i); //simple squared Sum

				// Update if the solution improves or not too loud
				if ( fnew <= FIT[i] && rand.nextDouble() < L ){ //Loudness with sLA, coolant to avoid overfitting 
                    // According to video increase pulse rate and reduce loudness here(https://www.youtube.com/watch?v=peqgggW-gcs)
					POP_SOL[i] = S[i];
					FIT[i] = fnew;
				}

				// Update the current best solution
				if ( fnew <= fmin ){
					B = POP_SOL[i];
					fmin = fnew;
					indexFinal = i;
				}
			} // end loop for BATS
			n_iter = n_iter + BATS;
		} // end loop for MAX
        // for ( int i = 0; i < BATS; i++ ){
        //     System.out.println(objective(POP_SOL[i]));
        // }

        System.out.println("------------");
		System.out.println("Number of evaluations : " + n_iter );
		System.out.println("Best = " + Arrays.toString(B) );
		System.out.println("fmin = " + fmin );
		return indexFinal;
	}
	/**
	 * Gets the free pes.
	 * 
	 * @return the free pes
	 */
	protected List<Integer> getFreePes() {
		return freePes;
	}
	
	
	/**
	 * Sets the free pes.
	 * 
	 * @param freePes the new free pes
	 */
	protected void setFreePes(List<Integer> freePes) {
		this.freePes = freePes;
	}
		
	/**
	 * Sets the used pes.
	 * 
	 * @param usedPes the used pes
	 */
	protected void setUsedPes(Map<String, Integer> usedPes) {
		this.usedPes = usedPes;
	}

	
	/**
	 * Sets the vm table.
	 * 
	 * @param vmTable the vm table
	 */
	protected void setVmTable(Map<String, Host> vmTable) {
		this.vmTable = vmTable;
	}
	@Override
	public boolean allocateHostForVm(Vm vm, Host host) {
		if (host.vmCreate(vm)) { // if vm has been succesfully created in the host
			getVmTable().put(vm.getUid(), host);

			int requiredPes = vm.getNumberOfPes();
			int idx = getHostList().indexOf(host);
			getUsedPes().put(vm.getUid(), requiredPes);
			getFreePes().set(idx, getFreePes().get(idx) - requiredPes);

			Log.formatLine(
					"%.2f: VM #" + vm.getId() + " has been allocated to the host #" + host.getId(),
					CloudSim.clock());
			return true;
		}

		return false;
	}
	@Override
	public List<Map<String, Object>> optimizeAllocation(List<? extends Vm> vmList) {
		// TODO Auto-generated method stub
		return null;
	}
	@Override
	public void deallocateHostForVm(Vm vm) {
		Host host = getVmTable().remove(vm.getUid());
		int idx = getHostList().indexOf(host);
		int pes = getUsedPes().remove(vm.getUid());
		if (host != null) {
			host.vmDestroy(vm);
			getFreePes().set(idx, getFreePes().get(idx) + pes);
		}
	}
	@Override
	public Host getHost(Vm vm) {
		return getVmTable().get(vm.getUid());
	}
	@Override
	public Host getHost(int vmId, int userId) {
		return getVmTable().get(Vm.getUid(userId, vmId));
	}
	/**
	 * Gets the vm table.
	 * 
	 * @return the vm table
	 */
	public Map<String, Host> getVmTable() {
		return vmTable;
	}
	
	/**
	 * Gets the used pes.
	 * 
	 * @return the used pes
	 */
	protected Map<String, Integer> getUsedPes() {
		return usedPes;
	}

}
