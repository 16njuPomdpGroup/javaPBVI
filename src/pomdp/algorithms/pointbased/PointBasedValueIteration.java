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

	/**
	 * ����㼯B
	 * �������B�����ȡ��b�������ĺ��������㼯B��Զ��
	 * @param vBeliefPoints ԭ�㼯
	 * @return �����㼯
	 */
	public BeliefStateVector<BeliefState> expandPBVI(BeliefStateVector<BeliefState> vBeliefPoints)
	{
		//������B��ԭ�ȵ�B�������Ѿ�������
    	BeliefStateVector<BeliefState> vExpanded = new BeliefStateVector<BeliefState>( vBeliefPoints );
    	
    	//��ʱ��������ŵ�ǰ���������b
    	BeliefState bsCurrent = null;
    	//��ʱ��������ŵõ�����Զb
    	BeliefState bsNext = null;
    	
    	//�����Ƿ���Ҫ����b
    	boolean bPrevious = m_pPOMDP.getBeliefStateFactory().cacheBeliefStates( false );
    	//����ķ�����������㼯��СС��100ʱ��ÿ�ζ�������һ����С;����100ʱ��ÿ������100��
    	int expandSize = 0;
    	if(vExpanded.size()<100)
    	{
    		expandSize = 2*vBeliefPoints.size();
    	}
    	else
    	{
    		expandSize = vBeliefPoints.size()+100;
    	}
    	//��ʼ����
    	while(vExpanded.size()<expandSize)
    	{
    		//�Ǵ������B�����ȡ��b������������Զ��̣����ͱ�׼PBVI��expand��ͬ
    		bsCurrent = vExpanded.elementAt(m_rndGenerator.nextInt(vExpanded.size()));
    		
    		//������Զ�ĺ�̣�action�����ȡһ����o��ȫ������
    		bsNext = m_pPOMDP.getBeliefStateFactory().computeRandomFarthestSuccessor( vBeliefPoints, bsCurrent );
    		if( ( bsNext != null ) && ( !vExpanded.contains( bsNext ) ) )
    		{
				vExpanded.add(bsCurrent, bsNext);//��������,bsNext�ǻ���bsCurrent�õ���
    		}
    	}
    	//���û�ԭ����ֵ���Ƿ�Ҫ����b
    	m_pPOMDP.getBeliefStateFactory().cacheBeliefStates( bPrevious );
    	
    	return vExpanded;
	}

	/**
	 * PBVI��ֵ����
	 * 1. ���ŵ㼯
	 * 2. ִ��backup����������ֵ����
	 * @param cIteration ����������
	 * @param dEpsilon ���ֵ
	 * @param dTargetValue ��Ҫ�ﵽ��ƽ���ۿۻر�ֵ��ADR��average discounted reward
	 */
	public void valueIteration(int cIteration, double dEpsilon,
			double dTargetValue) {
		/*
		����һ������״̬��������������ʼ�����ѳ�ʼ��ʼ���������������У�
		 */
		//maxRunningTime��numEvaluationsû���õ�  Ӧ��ɾ��
		BeliefStateVector<BeliefState> vBeliefPoints = new BeliefStateVector<BeliefState>();
		vBeliefPoints.add(null, m_pPOMDP.getBeliefStateFactory().getInitialBeliefState() );//��ʼ�������
		
		boolean done = false;//��ֹ����  �㼯���ٱ仯����ADR����
		for(int iIteration = 0; iIteration < cIteration && !done; ++iIteration)
		{
			/*
			1. ���ŵ㼯
			 */
			if(iIteration > 0)
			{
				Logger.getInstance().logln( "Expanding belief space" );
				
				int beliefPoints = vBeliefPoints.size();
				vBeliefPoints = expandPBVI(vBeliefPoints);//���ŵ㼯
				
				Logger.getInstance().logln( "Expanded belief space - |B| = " + vBeliefPoints.size() );
				
				if(beliefPoints == vBeliefPoints.size())//�㼯���ٷ����仯
				{
					done = true;
				}
			}

			/*
			2. ִ��backup����������ֵ����
			 */
			improveValueFunction(vBeliefPoints);
			Pair<Double, Double> pComputedADRs = new Pair<Double, Double>(new Double(0.0), new Double(0.0));
			/*
			����㼯���ڱ仯��ƽ���ۿۻر�ֵ�Ѿ����������������
			 */
			done = done || checkADRConvergence( m_pPOMDP, dTargetValue, pComputedADRs );//ADR����
			
			Logger.getInstance().logln( 
					"PBVI: Iteration " + iIteration + ","  +
					" |Vn| = " + m_vValueFunction.size() +
					" |B| = " + vBeliefPoints.size() +
					" simulated ADR " + ((Number) pComputedADRs.first()).doubleValue() +
					" " );
			Logger.getInstance().logln();
		}
	}

	/**
	 * ����ֵ����
	 * ������������ײ㿪ʼ�����϶Բ��е����������ִ��backup����������ֵ����
	 * @param vBeliefState ����״̬��������
	 */
	public void improveValueFunction(BeliefStateVector<BeliefState> vBeliefState)
	{
		ArrayList<ArrayList<BeliefState>> treeLevel = vBeliefState.getTreeLevelInfo();
		
		int levelSize = treeLevel.size();//�õ�����
		for(int i = levelSize - 1; i >= 0; --i)
		{
			ArrayList<BeliefState> level = treeLevel.get(i);//����õ�ÿһ�������㼯��
			
			m_itCurrentIterationPoints = level.iterator();
			while(m_itCurrentIterationPoints.hasNext())//����ÿһ��������
			{
				BeliefState bsCurrent = m_itCurrentIterationPoints.next();
				AlphaVector avCurrentMax = m_vValueFunction.getMaxAlpha( bsCurrent );//�õ���������
				AlphaVector avBackup = backup( bsCurrent );//���������
				
				double dBackupValue = avBackup.dotProduct( bsCurrent );
				double dValue = avCurrentMax.dotProduct( bsCurrent );
				double dDelta = dBackupValue - dValue;
				
				//������������Ż������µĦ�
				if(dDelta >= 0)
					m_vValueFunction.addPrunePointwiseDominated( avBackup );
			}
		}
	}

	/**
	 * �жϼ������ƽ���ۿۻر�ֵ�Ƿ�����
	 * @param pomdp ģ��
	 * @param dTargetADR ��Ҫ�ﵽ��ƽ���ۿۻر�ֵ
	 * @param pComputedADRs �������ƽ���ۿۻر�ֵ
	 * @return �Ƿ�����
	 */
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


  











