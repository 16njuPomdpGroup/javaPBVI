package pomdp;

import pomdp.algorithms.ValueIteration;
import pomdp.algorithms.pointbased.PointBasedValueIteration;
import pomdp.environments.POMDP;
import pomdp.utilities.Logger;

public class POMDPSolver {

	public static void main(String[] args) {
		String sPath = "Models/";// 得到model路径
		String sModelName = "hallway";// model名
		String sMethodName = "PBVI";//方法名
		Logger.getInstance().setOutput(true);//允许输出
		Logger.getInstance().setSilent(false);//允许输出到控制台
		try {
			String sOutputDir = "logs/POMDPSolver";// 输出路径
			String sFileName = sModelName + "_" + sMethodName + ".txt";// 输出文件名
			Logger.getInstance().setOutputStream(sOutputDir, sFileName);
		} catch (Exception e) {
			System.err.println(e);
		}

		POMDP pomdp = null;
		double dTargetADR = 100.0;// 目标平均折扣回报值，控制结束条件
		try {
			pomdp = new POMDP();
			pomdp.load(sPath + sModelName + ".POMDP");// 载入pomdp模型
			
			//输出最大回报值和最小回报值
    	    Logger.getInstance().logln("max is " + pomdp.getMaxR() + " min is " + pomdp.getMinR());
		} catch (Exception e) {
			Logger.getInstance().logln(e);
			e.printStackTrace();
			System.exit(0);
		}
		// 随机策略模拟，计算平均回报值
		pomdp.computeAverageDiscountedReward(2, 100,new RandomWalkPolicy(pomdp.getActionCount()));
		
		try
    	{
    		Logger.getInstance().setOutputStream( pomdp.getName() + "_" + sMethodName + ".txt" );
    	}
    	catch(Exception e)
    	{
    		System.err.println(e);
    	}
		
		//做了blind policy，获得PointBasedValueIteration
    	ValueIteration viAlgorithm = new PointBasedValueIteration(pomdp);
    	
    	int cMaxIterations = 400;
    	try
    	{
    		/* run POMDP solver */
    		viAlgorithm.valueIteration(cMaxIterations, 0.001, dTargetADR);
    		
    		/* compute the averaged return */
    		double dDiscountedReward = pomdp.computeAverageDiscountedReward( 2000, 150, viAlgorithm );
			Logger.getInstance().log( "POMDPSolver", 0, "main", "ADR = " + dDiscountedReward );
    	}
    	catch(Exception e)
    	{
    		Logger.getInstance().logln( e );
			e.printStackTrace();
    	}
    	catch(Error err)
    	{
    		Runtime rtRuntime = Runtime.getRuntime();
			Logger.getInstance().logln( 
					" POMDPSolver: " + err +
					" allocated " + ( rtRuntime.totalMemory() - rtRuntime.freeMemory() ) / 1000000 +
					" free " + rtRuntime.freeMemory() / 1000000 +
					" max " + rtRuntime.maxMemory() / 1000000 );
			Logger.getInstance().log( " Stack trace: " );
			err.printStackTrace();
    	}
	}
}
