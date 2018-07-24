package expressSubway;
import java.io.IOException;

import ilog.concert.*;
import ilog.cplex.*;

public class test1 {
	// sets
	public int T = 2;
	public int S = 15;
	public int h = 90;
	public int C = 2000;
	public int stop = 60;
	public double beta = 30;
	public double M = 2000.0;
 
	
	public test1(int T, int S, int h, int C, double beta, double M){
		this.T = T;
		this.S = S;
		this.h = h;
		this.C = C;
		this.beta = beta;
		this.M = M;
	}
	
	
	
	public void solveProb() throws IOException {
		
		int [][] ODs = {{0, 1}, {0, 2}, {0, 3}, {0, 4}, {0, 5}, {0, 6}, {0, 7}, {0, 8}, {0, 9}, {0, 10}, {0, 11}, {0, 12}, {0, 13}, {0, 14},
				   {1, 2}, {1, 3}, {1, 4}, {1, 5}, {1, 6}, {1, 7}, {1, 8}, {1, 9}, {1, 10}, {1, 11}, {1, 12}, {1, 13}, {1, 14},
				   {2, 3}, {2, 4}, {2, 5}, {2, 6}, {2, 7}, {2, 8}, {2, 9}, {2, 10}, {2, 11}, {2, 12}, {2, 13}, {2, 14}, 
				   {3, 4}, {3, 5}, {3, 6}, {3, 7}, {3, 8}, {3, 9}, {3, 10}, {3, 11}, {3, 12}, {3, 13}, {3, 14},
				   {4, 5}, {4, 6}, {4, 7}, {4, 8}, {4, 9}, {4, 10}, {4, 11}, {4, 12}, {4, 13}, {4, 14}, 
				   {5, 6}, {5, 7}, {5, 8}, {5, 9}, {5, 10}, {5, 11}, {5, 12}, {5, 13}, {5, 14},
				   {6, 7}, {6, 8}, {6, 9}, {6, 10}, {6, 11}, {6, 12}, {6, 13}, {6, 14},
				   {7, 8}, {7, 9}, {7, 10}, {7, 11}, {7, 12}, {7, 13}, {7, 14},
				   {8, 9}, {8, 10}, {8, 11}, {8, 12}, {8, 13}, {8, 14},
				   {9, 10}, {9, 11}, {9, 12}, {9, 13}, {9, 14},
				   {10, 11}, {10, 12}, {10, 13}, {10, 14},
				   {11, 12}, {11, 13}, {11, 14},
				   {12, 13}, {12, 14},
				   {13, 14}
				   };
		double [] pureTravel = {180, 60, 60, 120, 60, 60, 120, 120, 120, 120, 120, 120, 120, 120};
		int K = ODs.length;
		
		String address1 = "C:\\Users\\YJFKD\\Dropbox\\Express Optimization Individual\\Data\\ODTravel_time_local.csv";
		ReadLocalTravelTime rl = new ReadLocalTravelTime(K, address1);
		rl.readData();
		Double[] LocalTravel = rl.getLocalTravel();
		
		String address2 = "C:\\Users\\YJFKD\\Dropbox\\Express Optimization Individual\\Data\\ODTravel_time_express.csv";
		ReadExpressTravelTime re = new ReadExpressTravelTime(S, address2);
		re.readData();
		Double[][] ExpressTravel = re.getExpressTravel();
		
		String address3 = "C:\\Users\\YJFKD\\Dropbox\\Express Optimization Individual\\Data\\ODdemand_0222_18_2.csv";
		ReadTravelDemand rd = new ReadTravelDemand(K, address3);
		rd.readData();
		Double[] TravelDemand = rd.getTravelDemand();
		
		Double [] Total = new Double[K];
		for (int k=0; k<K; k++){
			Total[k] = TravelDemand[k] * LocalTravel[k];
		}

		
		try {
			IloCplex cplex = new IloCplex();
			cplex.setParam(IloCplex.Param.MIP.Tolerances.MIPGap, 0.05);
			
			// decision variables
			IloNumVar[][][] x = new IloNumVar[S][S][];
			for (int i=0; i<S; i++){
				for (int j=i+1; j<S; j++){
					x[i][j] = cplex.boolVarArray(T);
				}
			}
			IloNumVar[][][] z = new IloNumVar[S][S][];
			for (int i=0; i<S; i++){
				for (int j=i+1; j<S; j++){
					z[i][j] = cplex.boolVarArray(T);
				}
			}
			IloNumVar[][] alpha = new IloNumVar[S][];
			for (int i=0; i<S; i++){
				alpha[i] = cplex.boolVarArray(T+1);
			}
			IloIntVar[][][][] y = new IloIntVar[K][S][S][];
			for (int k=0; k<K; k++){
				for (int i=0; i<S; i++){
					for (int j=i+1; j<S; j++){
						y[k][i][j] = cplex.intVarArray(T, 0, (int) Double.MAX_VALUE);
					}
				}
			}
			IloIntVar[][] A = new IloIntVar[S][];
			for (int s=0; s<S; s++){
				A[s] = cplex.intVarArray(T+1, 0, (int) Double.MAX_VALUE);
			}
			IloIntVar[][] Q = new IloIntVar[K][];
			for (int k=0; k<K; k++){
				Q[k] = cplex.intVarArray(T, 0, (int) Double.MAX_VALUE);
			}
			IloIntVar[][] B = new IloIntVar[K][];
			for (int k=0; k<K; k++){
				B[k] = cplex.intVarArray(T, 0, (int) Double.MAX_VALUE);
			}
			IloIntVar[][] TravelDemand2 = new IloIntVar[K][];
			for (int k=0; k<K; k++){
				TravelDemand2[k] = cplex.intVarArray(T, 0, (int) Double.MAX_VALUE);
			}
			
			
			// objective function			
			IloNumExpr objective1 = cplex.constant(0);
			for (int k=0; k<K; k++){
				for (int t=0; t<T; t++){
					for (int i=0; i<S-1; i++){
						for (int j=i+1; j<S; j++){
							objective1 = cplex.sum(objective1, cplex.prod(stop, cplex.prod(y[k][i][j][t], j-i-1)));
						}
					}
				}
			}
			
			IloNumExpr objective2 = cplex.constant(0);
			for (int k=0; k<K; k++){
				for (int t=0; t<T; t++){
					objective2 = cplex.sum(objective2, cplex.sum(Q[k][t], cplex.negative(B[k][t])));
				}
			}
			
			IloNumExpr objective3 = cplex.constant(0);
			for (int k=0; k<K; k++){
				for (int t=0; t<T; t++){
					for (int i=0; i<S; i++){
						for (int j=i+1; j<S; j++){
							objective3 = cplex.sum(objective3, y[k][i][j][t]);
						}
					}
				}
			}

			IloNumExpr objectiveFunction = cplex.constant(0);
			objectiveFunction = cplex.sum(objective1, cplex.negative(cplex.prod(beta, objective2)));
//			objectiveFunction = objective3;
			
			// maximize objective
			cplex.addMaximize(objectiveFunction);
			
			// Constraints
			
			for (int k=0; k<K; k++){
				for (int t=1; t<T+1; t++){
					if (t < T){
						cplex.addLe(TravelDemand2[k][t], cplex.prod(cplex.sum(A[ODs[k][0]][t], cplex.negative(A[ODs[k][0]][t-1])), TravelDemand[k]));
						cplex.addGe(TravelDemand2[k][t], cplex.sum(cplex.prod(cplex.sum(A[ODs[k][0]][t], cplex.negative(A[ODs[k][0]][t-1])), TravelDemand[k]), -1));
					}
					else{
						cplex.addLe(TravelDemand2[k][0], cplex.prod(cplex.sum(A[ODs[k][0]][t], cplex.negative(A[ODs[k][0]][t-1])), TravelDemand[k]));
						cplex.addGe(TravelDemand2[k][0], cplex.sum(cplex.prod(cplex.sum(A[ODs[k][0]][t], cplex.negative(A[ODs[k][0]][t-1])), TravelDemand[k]), -1));
					}
				}
			}		
			
			for (int k=0; k<K; k++){
				for (int t=0; t<T; t++){
					for (int i=1; i<S-1; i++){
						IloNumExpr expr2 = cplex.constant(0);
						for (int j=i+1; j<S; j++){
							expr2 = cplex.sum(expr2, x[i][j][t]);
						}
						cplex.addLe(expr2, 1.0, "constraint (1)");
					}
				}
			}
			
			for (int t=0; t<T; t++){
				IloNumExpr expr3 = cplex.constant(0);
				for (int j=1; j<S; j++){
					expr3 = cplex.sum(expr3, x[0][j][t]);
				}
				cplex.addEq(expr3, 1, "constraint (2)");
			}
			
			for (int t=0; t<T; t++){
				IloNumExpr expr4 = cplex.constant(0);
				for (int i=0; i<S-1; i++){
					expr4 = cplex.sum(expr4, x[i][S-1][t]);
				}
				cplex.addEq(expr4, 1, "constraint (3)");
			}
			
			for (int t=0; t<T; t++){
				for (int i=1; i<S-1; i++){
					IloNumExpr expr6 = cplex.constant(0);
					for (int j=i+1; j<S; j++){
						expr6 =cplex.sum(expr6, x[i][j][t]);
					}
					IloNumExpr expr7 = cplex.constant(0);
					for (int j=0; j<i; j++){
						expr7 =cplex.sum(expr7, x[j][i][t]);
					}
					cplex.addEq(expr6, expr7, "constraint (4)");
				}
			}						
			
			for (int t=0; t<T; t++){
				for (int i=0; i<S-1; i++){
					IloNumExpr expr0 = cplex.constant(0);
					for (int j=i+1; j<S; j++){
						expr0 = cplex.sum(expr0, x[i][j][t]);
					}
					cplex.addEq(alpha[i][t], expr0, "constriant (5)");
				}
			}
			
			for (int t=0; t<T; t++){
				cplex.addEq(alpha[S-1][t], 1, "constraint (6a)");
			}
			
			for (int i=0; i<S; i++){
				cplex.addEq(alpha[i][T], alpha[i][0], "constraint (6b)");
			}
			
			for (int t=0; t<T; t++){
				for (int i=0; i<S-1; i++){
					for (int j=i+1; j<S; j++){
						cplex.addLe(z[i][j][t], alpha[i][t], "constraint (7)");
						cplex.addLe(z[i][j][t], alpha[j][t], "constraint (8)");
						cplex.addGe(z[i][j][t], cplex.sum(-1, cplex.sum(alpha[i][t], alpha[j][t])), "constraint (9)");
					}
				}
			}
						
			// Ensure for each OD pair, there exist a express service connect the origin and destination
			for (int i=0; i<S-1; i++){
				for (int j=i+1; j<S; j++){
					IloNumExpr expr5 = cplex.constant(0);
					for (int t=0; t<T; t++){
						expr5 = cplex.sum(expr5, z[i][j][t]);
					}
					cplex.addGe(expr5, 1, "constraint (10)");
				}
			}
			
/*			for (int i=0; i<S-1; i++){
				IloNumExpr make = cplex.constant(0);
				for (int t=0; t<T; t++){
					make = cplex.sum(make, alpha[i][t]);
				}
				cplex.addGe(make, 1);
			}*/
					
			
			for (int k=0; k<K; k++){
				for (int t=0; t<T; t++){
					cplex.addLe(B[k][t], Q[k][t], "linearization constraint 2");
					cplex.addLe(B[k][t], cplex.prod(M, z[ODs[k][0]][ODs[k][1]][t]), "linearization constraint 3");
//					cplex.addGe(B[k][t], cplex.sum(Q[k][t], cplex.negative(cplex.prod(M, cplex.sum(1, cplex.negative(z[ODs[k][0]][ODs[k][1]][t]))))), "constraint add 5");
				}
			}	
					
			for (int t=0; t<T; t++){
				for (int k=0; k<K; k++){
					IloNumExpr y1 = cplex.constant(0);
					IloNumExpr y2 = cplex.constant(0);
					for (int i=0; i<S-1; i++){
						if (i == ODs[k][0]){
							for (int j=i+1; j<S; j++){
								y1 = cplex.sum(y1, y[k][i][j][t]);
							}
						}
					}
					for (int j=1; j<S; j++){
						if (j == ODs[k][1]){
							for (int i=0; i<j; i++){
								y2 = cplex.sum(y2, y[k][i][j][t]);
							}
						}
					}
					/*cplex.addEq(y1, cplex.prod(Q[k][t], z[ODs[k][0]][ODs[k][1]][t]), "constraint (5)");
					cplex.addEq(y2, cplex.prod(Q[k][t], z[ODs[k][0]][ODs[k][1]][t]), "constraint (6)");*/
					cplex.addEq(y1, B[k][t], "constraint (11)");
					cplex.addEq(y2, B[k][t], "constraint (12)");
				}
			}
			
			for (int t=0; t<T; t++){
				for (int k=0; k<K; k++){
					for (int i=0; i<S-1; i++){
						for (int j=i+1; j<S; j++){
							cplex.addLe(y[k][i][j][t], cplex.prod(M, x[i][j][t]), "constraint (13)");
						}
					}
				}
			}
			
			for (int t=0; t<T; t++){
				for (int k=0; k<K; k++){
					for (int i=1; i<S-1; i++){
						if (i != ODs[k][0] && i != ODs[k][1]){
							IloNumExpr expr8 = cplex.constant(0);
							IloNumExpr expr9 = cplex.constant(0);
							for (int j=i+1; j<S; j++){
								expr8 = cplex.sum(expr8, y[k][i][j][t]);
							}
							for (int j=0; j<i; j++){
								expr9 = cplex.sum(expr9, y[k][j][i][t]);
							}
							cplex.addEq(expr8, expr9, "constraint (14)");
						}
					}
				}
			}
			
			for (int t=0; t<T; t++){
				for (int i=0; i<S-1; i++){
					for (int j=i+1; j<S; j++){
						IloNumExpr expr10 = cplex.constant(0);
						for (int k=0; k<K; k++){
							expr10 = cplex.sum(expr10, y[k][i][j][t]);
						}
						cplex.addLe(expr10, C, "constraint (15)");
					}
				}
			}	
			
			for (int t=0; t<T; t++){
				for (int k=0; k<K; k++){
					for (int i=0; i<S; i++){
						for (int j=i+1; j<S; j++){
							if (ODs[k][0] > i || ODs[k][1] < j){
								cplex.addEq(y[k][i][j][t], 0, "constraint (16)");
							}
						}
					}
				}
			}

			
			// Headway constraints
			cplex.addEq(A[0][0], 0, "constraint (17a)");
			
			for (int t=0; t<T+1; t++){
				for (int s=1; s<S; s++){
					IloNumExpr expr11 = cplex.constant(0);
					IloNumExpr expr12 = cplex.constant(0);
					for (int i=0; i<s; i++){
						expr11 = cplex.sum(expr11, pureTravel[i]);
					}
					for (int i=0; i<s; i++){
						expr12 = cplex.sum(expr12, cplex.prod(stop, alpha[i][t]));
					}
					cplex.addEq(A[s][t], cplex.sum(A[0][t], cplex.sum(expr11, expr12)), "constraint(17b)");
				}
			}
			
			for (int t=0; t<T; t++){
				for (int s=0; s<S; s++){
					cplex.addGe(cplex.sum(A[s][t+1], cplex.negative(A[s][t])), h, "constraint (18)");
				}
			}
			
			// Set minimum and maximum headway 
			for (int t=0; t<T; t++){
				cplex.addGe(cplex.sum(A[0][t+1], cplex.negative(A[0][t])), 60);
				cplex.addLe(cplex.sum(A[0][t+1], cplex.negative(A[0][t])), 180);
			}
						
			// Linear constraints
			for (int k=0; k<K; k++){
				for (int t=1; t<T+1; t++){
					if (t < T){
//						cplex.addEq(Q[k][t], cplex.prod(cplex.sum(A[ODs[k][0]][t], A[ODs[k][0]][t-1]), TravelDemand[k]), "linear constraint 1-1");
						cplex.addEq(Q[k][t], TravelDemand2[k][t], "linearization constraint 1-1");
					}
					else{
//						cplex.addEq(Q[k][0], cplex.prod(cplex.sum(A[ODs[k][0]][t], A[ODs[k][0]][t-1]), TravelDemand[k]), "linear constraint 1-2");
						cplex.addEq(Q[k][0], TravelDemand2[k][0], "linearization constraint 1-2");
					}
				}
			}	
			
			// add new cuts (transfer stations can not be skipped)
			for (int t=0; t<T; t++){
				cplex.addEq(alpha[1][t], 1);
				cplex.addEq(alpha[2][t], 1);
				cplex.addEq(alpha[4][t], 1);
				cplex.addEq(alpha[5][t], 1);
				cplex.addEq(alpha[9][t], 1);
				cplex.addEq(alpha[13][t], 1);
			}
			
			
			
			// solve problem
			if (cplex.solve()){
				System.out.println("Objective Value = "+cplex.getObjValue());
				System.out.println("Express Service Pattern ---------");
				
				// stop pattern
				for (int t=0; t<T; t++){
					for (int i=0; i<S; i++){
						System.out.print((int)Math.rint(cplex.getValue(alpha[i][t]))+",");
						}
					System.out.println();
					}
				
				// headway
				System.out.println("Train Headway ---------");
				for (int t=0; t<T+1; t++){
					for (int i=0; i<S; i++){
						System.out.print((int)Math.rint(cplex.getValue(A[i][t]))+",");
					}
					System.out.println();
				}
				
				// travel demand
				/*for (int k=0; k<K; k++){
					System.out.println("("+ODs[k][0]+", "+ODs[k][1]+"):"+cplex.getValue(TravelDemand2[k][0])+",");
				}*/

				}
			else {
				System.out.println("No solution!");
			}	
			cplex.end();
		}
		catch (IloException e) {
	         System.err.println("Concert exception caught: " + e);
	      }
	}

	
	
	
	// Main Function
	public static void main(String[] args) throws IOException {
		// TODO Auto-generated method stub
		long starTime= System.currentTimeMillis();
		
		// Solve Problem
		test1 Express1 = new test1(3, 15, 30, 2000, 100, 2000.0);
		Express1.solveProb();
		
		// Calculate running time
		long endTime = System.currentTimeMillis();
		long Time = endTime - starTime;
		System.out.println("Running Time: " + Time/1000 + "s");
	}
}
