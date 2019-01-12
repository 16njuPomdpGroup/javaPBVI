package pomdp.algorithms;
/*
 * Created on May 6, 2005
 *
 * To change the template for this generated file go to
 * Window&gt;Preferences&gt;Java&gt;Code Generation&gt;Code and Comments
 */

/**
 * @author shanigu
 *
 * To change the template for this generated type comment go to
 * Window&gt;Preferences&gt;Java&gt;Code Generation&gt;Code and Comments
 */

import java.util.Iterator;
import java.util.Vector;
import java.util.Map.Entry;

import pomdp.environments.POMDP;
import pomdp.utilities.AlphaVector;
import pomdp.utilities.BeliefState;
import pomdp.utilities.Logger;
import pomdp.utilities.RandomGenerator;
import pomdp.valuefunction.LinearValueFunctionApproximation;
import pomdp.valuefunction.MDPValueFunction;

public abstract class ValueIteration extends PolicyStrategy{
	protected MDPValueFunction m_vfMDP;
	protected LinearValueFunctionApproximation m_vValueFunction;
	protected POMDP m_pPOMDP;
	protected int m_cStates;
	protected int m_cActions;
	protected int m_cObservations;
	protected double m_dGamma;
	protected AlphaVector m_avMaxValues;
	protected final static double MIN_INF = Double.NEGATIVE_INFINITY;
	protected final static double MAX_INF = Double.POSITIVE_INFINITY;
	protected boolean m_bConverged;
	//ε 0.001
	protected double m_dEpsilon = 0.001;
	protected long m_cElapsedExecutionTime;
	protected long m_cCPUExecutionTime;
	protected long m_cDotProducts;
	protected int m_cValueFunctionChanges;
	protected boolean m_bTerminate;
	
	protected long m_cTimeInBackup;
	protected long m_cTimeInHV;
	protected long m_cTimeInV;
	protected long m_cAlphaVectorNodes; 
	
	protected static int g_cTrials = 500;
	protected static int g_cStepsPerTrial = 100;
	
	//没有用到的变量。没有地方来设置它
	protected static String m_sBlindPolicyValueFunctionFileName = null;
	
	protected RandomGenerator m_rndGenerator;	
	

	public ValueIteration( POMDP pomdp ){
		m_pPOMDP = pomdp;
		m_cStates = m_pPOMDP.getStateCount();
		m_cActions = m_pPOMDP.getActionCount();
		m_cObservations = m_pPOMDP.getObservationCount();
		m_dGamma = m_pPOMDP.getDiscountFactor();
		m_avMaxValues = null;
		m_bConverged = false;
		m_cElapsedExecutionTime = 0;
		m_cCPUExecutionTime = 0;
		//m_dFilteredADR = 0.0;
		m_cDotProducts = 0;
		
		m_cTimeInBackup = 0;
		m_cTimeInHV = 0;
		m_cTimeInV = 0;
		m_cAlphaVectorNodes = 0;
		
		m_bTerminate = false;
		
		//仅随机数生成器
		m_rndGenerator = new RandomGenerator( "ValueIteration" );
		
		//m_dEpsilon: ε 0.001
		m_vValueFunction = new LinearValueFunctionApproximation( m_dEpsilon, true );
		m_vfMDP = pomdp.getMDPValueFunction();

		//computeStepsPerTrial();
		initValueFunctionUsingBlindPolicy();

	}
	
	public abstract void valueIteration(int cMaxSteps, double dEpsilon, double dTargetValue);
	
	private double findMaxAlphas( int iAction, BeliefState bs, LinearValueFunctionApproximation vValueFunction, AlphaVector[] aNext ) {
		AlphaVector avAlpha = null;
		int iObservation = 0;
		double dSumValues = 0.0, dValue = 0, dProb = 0.0, dSumProbs = 0.0;
		BeliefState bsSuccessor = null;
		
		boolean bCache = m_pPOMDP.getBeliefStateFactory().isCachingBeliefStates();
		for( iObservation = 0 ; iObservation < m_cObservations ; iObservation++ ){//迭代每一个观察，获得最大期望值，不包括立即回报值
			dProb = bs.probabilityOGivenA( iAction, iObservation );//计算执行动作a得到观察o的概率
			dSumProbs += dProb;
			if( dProb > 0.0 ){
				bsSuccessor = bs.nextBeliefState( iAction, iObservation );//计算b、a、o的后继信念点
				avAlpha = vValueFunction.getMaxAlpha( bsSuccessor );//后继信念点在值函数中值最大时所对应的向量
				dValue = avAlpha.dotProduct( bsSuccessor );//后继信念点在值函数中对应的最大值
				dSumValues += dValue * dProb;//加权求和
			}
			else{
				avAlpha = vValueFunction.getLast();
			}
			aNext[iObservation] = avAlpha;//每一个0迭代一部以后对应的最大向量
			
		}
		
		//式F10
		//和标准backup中的式子不一样
		dSumValues /= dSumProbs; //in case due to rounding there is an error and probs do not exactly sum to 1
		dSumValues *= m_pPOMDP.getDiscountFactor();
		dSumValues += m_pPOMDP.immediateReward( bs, iAction ); //加上立即回报
		
		m_pPOMDP.getBeliefStateFactory().cacheBeliefStates( bCache );
		
		return dSumValues;
	}
	
	private AlphaVector G( int iAction, LinearValueFunctionApproximation vValueFunction, AlphaVector[] aNext) {
		AlphaVector avAlpha = null, avG = null, avSum = null, avResult = null;
		int iObservation = 0;
		
		//遍历o，把α相加
		for( iObservation = 0 ; iObservation < m_cObservations ; iObservation++ ){
			avAlpha = aNext[iObservation];
			avG = avAlpha.G( iAction, iObservation );
				//Logger.getInstance().logln( iObservation + ") " + avAlpha + ", " + avG );
			if( avSum == null )
				avSum = avG.copy();
			else
				avSum.accumulate( avG );
		}
		
		//增加立即回报
		avResult = avSum.addReward( iAction );
		avResult.setAction( iAction );

		avSum.release();
		return avResult;
	}
	
	public AlphaVector backup( BeliefState bs ){
		return backup( bs, m_vValueFunction );
	}
	
	public AlphaVector backup( BeliefState bs, LinearValueFunctionApproximation vValueFunction ){
		
		return  backupTauBased( bs, vValueFunction);
	}
	
	protected AlphaVector backupTauBased( BeliefState bs, LinearValueFunctionApproximation vValueFunction){
		double dValue = 0.0, dMaxValue = Double.NEGATIVE_INFINITY;
		int iAction = 0, iMaxAction = -1;
		AlphaVector[] aNext = null, aBest = null;
	
		Vector<AlphaVector[]> vWinners = new Vector<AlphaVector[]>();//可能因为相同，而有多个，所有弄个vector
		Vector<Integer> vWinnersActions = new Vector<Integer>();//最优action
	
		for( iAction = 0 ; iAction < m_cActions ; iAction++ ){//每个action最优α
			aNext = new AlphaVector[m_cObservations];//aNext存放由b、a、V，找到每个o对应的最大α
				//返回值为F10，类似b、a固定时，迭代一步后计算得到的最大value，在每个o迭代一步后对应的最大α条件下
			dValue = findMaxAlphas( iAction, bs, vValueFunction, aNext );
			
			if( dValue > dMaxValue ){
				dMaxValue = dValue;
				vWinners.clear();
				vWinnersActions.clear();
			}
			if( dValue == dMaxValue ){
				aBest = aNext;//aBest放的是最佳action对应的aNext
		                      //b、a、o固定时，最佳action下，每个o迭代一步后对应的最大α
				iMaxAction = iAction;
				vWinners.add( aBest );//记录取值最大的动作所对应的向量
				vWinnersActions.add( iMaxAction );//记录取值最大的动作
			}
		}
		
		//如果有多个最佳action，随机取一个
		int idx = m_rndGenerator.nextInt( vWinners.size() );
		aBest = vWinners.elementAt( idx );
		iMaxAction = vWinnersActions.elementAt( idx );
		//计算出更新后的α
		AlphaVector avMax = G( iMaxAction, vValueFunction, aBest );
		avMax.setWitness( bs );
		
		return avMax;
	}
	protected double getMaxMinR(){
		return m_pPOMDP.getMaxMinR();
	}
	
	//使用blind策略，初始化值函数V LinearValueFunctionApproximation
	protected void initValueFunctionUsingBlindPolicy(){
		m_vValueFunction.clear();
		
		int iAction = 0, iState = 0, iEndState = 0, iIteration = 0;
		double dValue = 0.0, dNewValue = 0.0, dReward = 0.0, dDiff = 0.0, dMaxDiff = 0.0, dTr = 0.0, dSum = 0.0;
		AlphaVector av = null;
		AlphaVector avNext = null;
		double dMaxResidual = MAX_INF;
		Entry<Integer,Double> entry = null;
		Iterator<Entry<Integer,Double>> itNonZero = null;
		LinearValueFunctionApproximation vMin = new LinearValueFunctionApproximation( m_dEpsilon, false );
		//初始化为最小值
		initValueFunctionToMin( vMin );
		
		//获得状态概率均一致的b
		BeliefState bsUniform = m_pPOMDP.getBeliefStateFactory().getUniformBeliefState();
		
		if( m_sBlindPolicyValueFunctionFileName != null ){
			try{
				m_vValueFunction.load( m_sBlindPolicyValueFunctionFileName, m_pPOMDP );
				Logger.getInstance().logln( "Blind policy loaded successfully" );
				return;
			}
			catch( Exception e ){
				Logger.getInstance().logln( "Could not load blind policy - " + e );
			}
		}
		Logger.getInstance().logln( "Begin blind policy computation  " + m_cActions + " actions" );
		
		//MDP值迭代至收敛！！！
		//遍历action
		//计算F3，对每个state的值，取最大的，并记录对应的action
		for( iAction = 0 ; iAction < m_cActions ; iAction++ ){
			//计算F2，MDP，某action下，对所有state进行值迭代至收敛
			av = vMin.elementAt( 0 ); //取出第一个α，其实就只有一个α
			//av = m_pPOMDP.newAlphaVector();
			//Logger.getInstance().logln( "Initial " + av );
			iIteration = 0;
			dMaxResidual = MAX_INF;
			//持续迭代至收敛，即迭代前后最大value的差值，<=0.1
			while( dMaxResidual > 0.1 ){
				avNext = av.newAlphaVector();
				dMaxDiff = 0.0;
				//遍历state
				//MDP，对所有状态进行一步值迭代，然后把迭代后值，存入新α avNext
				//state 1
				for( iState = 0 ; iState < m_cStates ; iState++ ){
					//-------计算F1（S），MDP中一步值迭代
					dSum = 0.0;
					//获得以这个state action开始的转换
					//state2
					itNonZero = m_pPOMDP.getNonZeroTransitions( iState, iAction );
					while( itNonZero.hasNext() ){
						entry = itNonZero.next();
						iEndState = entry.getKey().intValue();
						dTr = entry.getValue().doubleValue();//转换的概率值
						dValue = av.valueAt( iEndState );//α在这个状态的的value
						dSum += dTr * dValue;//
					}
					dReward = m_pPOMDP.R( iState, iAction );
					dNewValue = dReward + dSum * m_dGamma;
					//-------
					//把MDP对一个state的一步迭代后值存入新的α
					avNext.setValue( iState, dNewValue );
					
					//更新一步迭代后，差值最大的state
					dDiff = Math.abs( dNewValue - av.valueAt( iState ) );
					if( dDiff > dMaxDiff ){
						dMaxDiff = dDiff;
					}
				}
				
				dMaxResidual = dMaxDiff;
				avNext.finalizeValues();
				//新α替代老α
				av = avNext;
				
				iIteration++;
				
				if( iIteration % 10 == 0 )
					Logger.getInstance().log( "." );
			}
						
			av.setWitness( bsUniform );
			av.setAction( iAction );//设置对应的action
			m_vValueFunction.addPrunePointwiseDominated( av );//这里会取出值最大的α
			Logger.getInstance().logln( "Done action " + iAction +
					" after " + iIteration + " iterations |V| = " + m_vValueFunction.size() );
		}		
		Logger.getInstance().logln( "Done blind policy" );
		
		if( m_sBlindPolicyValueFunctionFileName != null ){
			try{
				m_vValueFunction.save( m_sBlindPolicyValueFunctionFileName );
				Logger.getInstance().logln( "Blind policy saved successfully" );
				return;
				
			}
			catch( Exception e ){
				Logger.getInstance().logln( "Could not save blind policy - " + e );
			}
		}
		
	}
	
	protected void initValueFunctionToMin( LinearValueFunctionApproximation vValueFunction ){
		Logger.getInstance().logln( "Init value function to min" );
		double dMinValue = getMaxMinR();
		double dDefaultValue = dMinValue / ( 1 - m_dGamma );

 		BeliefState bsUniform = m_pPOMDP.getBeliefStateFactory().getUniformBeliefState();
		Logger.getInstance().logln( "Min R value = " + dMinValue + " init value = " + dDefaultValue );
	
		AlphaVector avMin = null;
		avMin = m_pPOMDP.newAlphaVector();
		avMin.setAllValues( dDefaultValue );
		avMin.finalizeValues();
		avMin.setWitness( bsUniform );
		vValueFunction.add( avMin );
	}

	@SuppressWarnings({ "rawtypes", "unchecked" })
	protected Iterator backwardIterator( Vector vElements ){
		Vector vBackward = new Vector();
		int iElement = 0, cElements = vElements.size();
		for( iElement = cElements - 1 ; iElement >= 0 ; iElement-- ){
			vBackward.add( vElements.get( iElement ) );
		}
		return vBackward.iterator();
	}

	
	
}
