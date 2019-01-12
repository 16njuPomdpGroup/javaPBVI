package pomdp.algorithms.pointbased;

import java.util.ArrayList;
import java.util.Iterator;

import pomdp.algorithms.ValueIteration;
import pomdp.environments.POMDP;
import pomdp.utilities.AlphaVector;
import pomdp.utilities.BeliefState;
import pomdp.utilities.BeliefStateVector;
import pomdp.utilities.Logger;
import pomdp.utilities.Pair;


public class PointBasedValueIteration extends ValueIteration
{
     protected Iterator<BeliefState> m_itCurrentIterationPoints;
     protected boolean m_bSingleValueFunction = true;
 	 protected boolean m_bRandomizedActions;
 	protected double m_dFilteredADR = 0.0;
 	 
 	public PointBasedValueIteration( POMDP pomdp ){
		super(pomdp);
		
		m_itCurrentIterationPoints = null;
		m_bRandomizedActions = true;
	}

	public PointBasedValueIteration( POMDP pomdp, boolean bRandomizedActionExpansion ){
		super(pomdp);
		
		m_itCurrentIterationPoints = null;
		m_bRandomizedActions = bRandomizedActionExpansion;
	}
	
	public BeliefStateVector<BeliefState> expandPBVI(BeliefStateVector<BeliefState> vBeliefPoints)
	{
		//扩充后的B，原先的B中内容已经在这里
    	BeliefStateVector<BeliefState> vExpanded = new BeliefStateVector<BeliefState>( vBeliefPoints );
    	
    	//临时变量，存放当前用来扩充的b
    	BeliefState bsCurrent = null;
    	//临时变量，存放得到的最远b
    	BeliefState bsNext = null;
    	
    	//设置是否需要缓存b
    	boolean bPrevious = m_pPOMDP.getBeliefStateFactory().cacheBeliefStates( false );
    	//扩充的方法，在信念点集大小小于100时，每次都是扩充一倍大小;大于100时，每次扩充100个
    	int expandSize = 0;
    	if(vExpanded.size()<100)
    	{
    		expandSize = 2*vBeliefPoints.size();
    	}
    	else
    	{
    		expandSize = vBeliefPoints.size()+100;
    	}
    	//开始扩充
    	while(vExpanded.size()<expandSize)
    	{
    		//是从扩充后B中随机取个b，计算它的最远后继！！和标准PBVI中expand不同
    		bsCurrent = vExpanded.elementAt(m_rndGenerator.nextInt(vExpanded.size()));
    		
    		//计算最远的后继，action是随机取一个，o是全遍历的
    		bsNext = m_pPOMDP.getBeliefStateFactory().computeRandomFarthestSuccessor( vBeliefPoints, bsCurrent );
    		if( ( bsNext != null ) && ( !vExpanded.contains( bsNext ) ) )
    		{
				vExpanded.add(bsCurrent, bsNext);//这里的添加,bsNext是基于bsCurrent得到的
    		}
    	}
    	//设置回原来的值，是否要缓存b
    	m_pPOMDP.getBeliefStateFactory().cacheBeliefStates( bPrevious );
    	
    	return vExpanded;
	}

	
	public void valueIteration(int cIteration, double dEpsilon,
			double dTargetValue) {
		//maxRunningTime和numEvaluations没有用到  应该删除
		BeliefStateVector<BeliefState> vBeliefPoints = new BeliefStateVector<BeliefState>();
		vBeliefPoints.add(null, m_pPOMDP.getBeliefStateFactory().getInitialBeliefState() );//初始化信念点
		
		boolean done = false;//终止条件  点集不再变化或者ADR收敛
		for(int iIteration = 0; iIteration < cIteration && !done; ++iIteration)
		{
			if(iIteration > 0)
			{
				Logger.getInstance().logln( "Expanding belief space" );
				
				int beliefPoints = vBeliefPoints.size();
				vBeliefPoints = expandPBVI(vBeliefPoints);//扩张点集
				
				Logger.getInstance().logln( "Expanded belief space - |B| = " + vBeliefPoints.size() );
				
				if(beliefPoints == vBeliefPoints.size())//点集不再发生变化
				{
					done = true;
				}
			}
			
			improveValueFunction(vBeliefPoints);
			Pair<Double, Double> pComputedADRs = new Pair<Double, Double>(new Double(0.0), new Double(0.0));
			done = done || checkADRConvergence( m_pPOMDP, dTargetValue, pComputedADRs );//ADR收敛
			
			Logger.getInstance().logln( 
					"PBVI: Iteration " + iIteration + ","  +
					" |Vn| = " + m_vValueFunction.size() +
					" |B| = " + vBeliefPoints.size() +
					" simulated ADR " + ((Number) pComputedADRs.first()).doubleValue() +
					" " );
			Logger.getInstance().logln();
		}
	}
	
	public void improveValueFunction(BeliefStateVector<BeliefState> vBeliefState)
	{
		ArrayList<ArrayList<BeliefState>> treeLevel = vBeliefState.getTreeLevelInfo();
		
		int levelSize = treeLevel.size();//得到层数
		for(int i = levelSize - 1; i >= 0; --i)
		{
			ArrayList<BeliefState> level = treeLevel.get(i);//逆序得到每一层的信念点集合
			
			m_itCurrentIterationPoints = level.iterator();
			while(m_itCurrentIterationPoints.hasNext())//更新每一层的信念点
			{
				BeliefState bsCurrent = m_itCurrentIterationPoints.next();
				AlphaVector avCurrentMax = m_vValueFunction.getMaxAlpha( bsCurrent );//得到最大的向量
				AlphaVector avBackup = backup( bsCurrent );//更新信念点
				
				double dBackupValue = avBackup.dotProduct( bsCurrent );
				double dValue = avCurrentMax.dotProduct( bsCurrent );
				double dDelta = dBackupValue - dValue;
				
				//如果有提升，才会增加新的α
				if(dDelta >= 0)
					m_vValueFunction.addPrunePointwiseDominated( avBackup );
			}
		}
	}

	protected boolean checkADRConvergence( POMDP pomdp, double dTargetADR, Pair<Double,Double> pComputedADRs ){
		double dSimulatedADR = 0.0;
		boolean bConverged = false;
		
		pComputedADRs.setFirst( new Double( 0.0 ) );
		pComputedADRs.setSecond( new Double( 0.0 ) );
		
		if( pomdp != null && g_cTrials > 0 ){
			dSimulatedADR = pomdp.computeAverageDiscountedReward( g_cTrials, g_cStepsPerTrial, this );
			
			if( m_dFilteredADR == 0.0 ){
				m_dFilteredADR = dSimulatedADR;
			}
			else{
				m_dFilteredADR = ( m_dFilteredADR + dSimulatedADR ) / 2;
				if( m_dFilteredADR >= dTargetADR )
					bConverged = true;
			}
			
			if( pComputedADRs != null ){
				pComputedADRs.setFirst( new Double( dSimulatedADR ) );
				pComputedADRs.setSecond( new Double( m_dFilteredADR ) );
			}						
		}
		return bConverged || m_bTerminate;
	}
	
	public int getAction(BeliefState bsCurrent) {
		return m_vValueFunction.getBestAction(bsCurrent);
	}
}


  











