package pomdp.environments;

import java.io.IOException;
import java.io.Serializable;
import java.util.AbstractCollection;
import java.util.Collection;
import java.util.Iterator;
import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;
import java.util.Vector;

import pomdp.algorithms.PolicyStrategy;
import pomdp.utilities.AlphaVector;
import pomdp.utilities.BeliefState;
import pomdp.utilities.BeliefStateFactory;
import pomdp.utilities.InvalidModelFileFormatException;
import pomdp.utilities.Logger;
import pomdp.utilities.POMDPLoader;
import pomdp.utilities.RandomGenerator;
import pomdp.utilities.SparseTabularFunction;
import pomdp.utilities.TabularAlphaVector;
import pomdp.utilities.datastructures.Function;
import pomdp.utilities.datastructures.TabularFunction;
import pomdp.valuefunction.MDPValueFunction;

public class POMDP implements Serializable {

	protected final static double MAX_INF = Double.POSITIVE_INFINITY;
	protected final static double MIN_INF = Double.NEGATIVE_INFINITY;

	private static final long serialVersionUID = -231630700034970161L;
	protected Function m_fTransition;// T:状态转移函数，s下执行a到s'的概率
	protected Function m_fReward;// R:回报值函数，s下执行a的回报
	protected Function m_fObservation;// O:a后得到s时，观察到o的概率
	protected Function m_fStartState;// b.:初始时，各状态的概率
	// S
	protected Vector<String> m_vStateNames;// 状态名称向量
	protected Map<String, Integer> m_mStates;// 存储状态与其索引
	// A
	protected Map<String, Integer> m_mActionIndexes;// 存储动作与其索引
	protected Vector<String> m_vActionNames;// 动作名称向量
	// O
	protected Map<String, Integer> m_mObservations;// 存储观察值和对应的index号
	protected int m_cStates;// 状态个数
	protected int m_cActions;// 动作个数
	protected int m_cObservations;// 观察值个数
	protected double m_dGamma; // 折扣参数

	protected static int g_sMaxTabularSize = 3000;// 控制函数类型的分界点
	protected Vector<Integer> m_vTerminalStates;// 终止状态向量
	protected Vector<Integer> m_vObservationStates;// 保存观察值敏感状态向量
	protected double[][] m_adStoredRewards;// 已计算的状态动作回报值
	protected double[] m_adMinActionRewards;// 保存每个动作的最小回报值

	protected RandomGenerator m_rndGenerator;
	protected long m_iRandomSeed;
	protected String m_sName;// 模型名称
	protected RewardType m_rtReward;// 回报值类型
	protected BeliefStateFactory m_bsFactory;// 信念工厂
	protected MDPValueFunction m_vfMDP;
	protected double m_dMinReward;// 最小回报值

	public enum RewardType {
		StateActionState, ActionEndState, StateAction, State;
	}

	public POMDP() {
		m_fTransition = null;
		m_fReward = null;
		m_fObservation = null;
		m_mActionIndexes = null;
		m_vActionNames = null;
		m_mStates = null;
		m_mObservations = null;
		m_fStartState = null;
		m_cStates = 0;
		m_cActions = 0;
		m_cObservations = 0;
		m_dGamma = 0.95;
		m_vTerminalStates = null;
		m_vObservationStates = new Vector<Integer>();
		m_adMinActionRewards = null;
		m_iRandomSeed = 0;
		m_rndGenerator = new RandomGenerator("POMDP");
		m_sName = "";
		m_rtReward = RewardType.StateAction;// 回报值类型设为状态动作

		m_bsFactory = null;
		m_vfMDP = null;
		m_dMinReward = 0.0;// Double.POSITIVE_INFINITY;
	}

	public Function getM_FReward() {
		return m_fReward;
	}

	public void load(String sFileName) throws IOException,
			InvalidModelFileFormatException {
		// 取模型名字
		m_sName = sFileName.substring(sFileName.lastIndexOf("/") + 1,
				sFileName.lastIndexOf("."));
		// 从文件中加载
		POMDPLoader p = new POMDPLoader(this);
		p.load(sFileName);
		if (m_rtReward == RewardType.StateActionState)
			initStoredRewards();

		initBeliefStateFactory();// 初始化信念工厂
		m_vfMDP = new MDPValueFunction(this, 0.0);// 实例化MDP值函数

		Logger.getInstance().logln();
	}

	public BeliefStateFactory getBeliefStateFactory() {
		return m_bsFactory;
	}

	public MDPValueFunction getMDPValueFunction() {
		if (m_vfMDP == null) {
			m_vfMDP = new MDPValueFunction(this, 0.0);
		}
		return m_vfMDP;
	}

	public void resetMDPValueFunction() {
		m_vfMDP = new MDPValueFunction(this, 0.0);
	}

	protected void initStoredRewards() {
		m_adStoredRewards = new double[m_cStates][m_cActions];
		int iState = 0, iAction = 0;
		for (iState = 0; iState < m_cStates; iState++) {
			for (iAction = 0; iAction < m_cActions; iAction++) {
				m_adStoredRewards[iState][iAction] = MIN_INF;
			}
		}
	}

	/**
	 * T Function，求一个转换的概率
	 * 
	 * @param iState1
	 * @param iAction
	 * @param iState2
	 * @return
	 */
	public double tr(int iState1, int iAction, int iState2) {
		return m_fTransition.valueAt(iState1, iAction, iState2);
	}

	/**
	 * Immediate reward function of the POMDP. Checks reward type
	 * (R(s,a,s'),R(s,a) or R(s)) before accessing the reward function.
	 * 
	 * @param iStartState
	 * @param iAction
	 * @param iEndState
	 * @return immediate reward
	 */
	public double R(int iStartState, int iAction, int iEndState) {
		double dReward = 0.0;
		if (m_rtReward == RewardType.StateActionState)
			dReward = m_fReward.valueAt(iStartState, iAction, iEndState);// 只会调用这个
		else if (m_rtReward == RewardType.StateAction)
			dReward = m_fReward.valueAt(iStartState, iAction);
		else if (m_rtReward == RewardType.ActionEndState)
			dReward = m_fReward.valueAt(iAction, iEndState);
		else if (m_rtReward == RewardType.State)
			dReward = m_fReward.valueAt(iStartState);
		return dReward;
	}

	/**
	 * Immediate reward function of the POMDP.
	 * 
	 * @param iStartState
	 * @return immediate reward
	 */
	public double R(int iStartState) {
		double dReward = 0.0;
		if (m_rtReward == RewardType.State)
			dReward = m_fReward.valueAt(iStartState);
		return dReward;
	}

	/**
	 * Immediate reward function of the POMDP of the form R(s,a). If reward
	 * structure is of the form R(s,a,s') it sums over all possible s'.
	 * 
	 * @param iStartState
	 * @param iAction
	 * @return immediate reward
	 */
	public double R(int iStartState, int iAction) {// 状态iStartState执行动作iAction的回报
		int iEndState = 0;
		double dReward = 0.0, dSumReward = 0.0;
		double dTr = 0.0;
		Iterator<Entry<Integer, Double>> itNonZeroEntries = null;
		Entry<Integer, Double> e = null;

		if (m_rtReward == RewardType.StateAction)
			dReward = m_fReward.valueAt(iStartState, iAction);
		else if (m_rtReward == RewardType.State)
			dReward = m_fReward.valueAt(iStartState);
		else if (m_rtReward == RewardType.StateActionState) {

			dReward = m_adStoredRewards[iStartState][iAction];
			if (dReward == MIN_INF) {

				dSumReward = m_fReward.valueAt(iStartState, iAction);
				if (dSumReward == 0) {
					// 如果按结束状态区分回报值，则按概率求和
					itNonZeroEntries = m_fReward.getNonZeroEntries(iStartState,
							iAction);
					if (itNonZeroEntries != null) {
						while (itNonZeroEntries.hasNext()) {
							e = itNonZeroEntries.next();
							iEndState = ((Number) e.getKey()).intValue();// 求转换结束状态
							dReward = ((Number) e.getValue()).doubleValue();// 求转换的回报
							dTr = tr(iStartState, iAction, iEndState);// 求转换的概率
							if (dTr > 0)
								dSumReward += dReward * dTr;
						}
					}
				}

				m_adStoredRewards[iStartState][iAction] = dSumReward;

				dReward = dSumReward;
			}

		}

		return dReward;
	}

	public double O(int iAction, int iEndState, int iObservation) {
		return m_fObservation.valueAt(iAction, iEndState, iObservation);
	}

	// 根据索引得到action名字
	public String getActionName(int iAction) {
		if (m_vActionNames != null) {
			return (m_vActionNames.get(new Integer(iAction))).toString();
		}
		return "" + iAction;
	}

	public int getActionIndex(String sAction) {
		if (m_mActionIndexes != null) {
			Object oIdx = m_mActionIndexes.get(sAction);
			if (oIdx != null) {
				return ((Integer) oIdx).intValue();
			}
		}
		try {
			return Integer.parseInt(sAction);
		} catch (NumberFormatException e) {
			return -1;
		}
	}

	public String getObservationName(int iObservation) {
		return iObservation + "";
	}

	// 根据索引得到state名字
	public String getStateName(int iState) {
		if (m_vStateNames != null)
			return (String) m_vStateNames.elementAt(iState);
		return iState + "";
	}

	public int getStateIndex(String sState) {// 得到m_mStates中相应状态索引
		Object oIdx = null;
		if (m_mStates != null)
			oIdx = m_mStates.get(sState);
		if (oIdx != null) {
			return ((Integer) oIdx).intValue();
		} else {
			try {
				return Integer.parseInt(sState);
			} catch (NumberFormatException e) {
				return -1;
			}
		}
	}

	public int getObservationIndex(String sObservation) {
		Object oIdx = null;
		if (m_mObservations != null)
			oIdx = m_mObservations.get(sObservation);
		if (oIdx != null) {
			return ((Integer) oIdx).intValue();
		} else {
			try {
				return Integer.parseInt(sObservation);
			} catch (NumberFormatException e) {
				return -1;
			}
		}
	}

	public void setTransition(int iStartState, int iAction, int iEndState,
			double dTr) {
		m_fTransition.setValue(iStartState, iAction, iEndState, dTr);// 设置状态转移函数
	}

	public void setObservation(int iAction, int iEndState, int iObservation,
			double dValue) {
		m_fObservation.setAllValues(iAction, iEndState, iObservation, dValue);// 设置观察值函数
	}

	public void setDiscountFactor(double dGamma) {// 设置折扣参数
		m_dGamma = dGamma;
	}

	public void setStateCount(int cStates) {// 设置状态数量
		m_cStates = cStates;
	}

	public void addState(String sState) {// 添加状态
		if (m_mStates == null) {
			m_mStates = new TreeMap<String, Integer>();
			m_vStateNames = new Vector<String>();
		}

		m_mStates.put(sState, m_cStates);// 将状态放入m_mStates中
		m_vStateNames.add(sState);// 将状态放入向量m_vStateNames中
		m_cStates++;
	}

	public void setActionCount(int cActions) {// 设置动作数量
		m_cActions = cActions;
	}

	public void addAction(String sAction) {// 添加动作
		if (m_mActionIndexes == null) {
			m_mActionIndexes = new TreeMap<String, Integer>();
			m_vActionNames = new Vector<String>();
		}

		m_mActionIndexes.put(sAction, m_cActions);// 将动作放入m_mActionIndexes中
		m_vActionNames.add(sAction);// 将动作放入向量m_vActionNames中
		m_cActions++;
	}

	public void setObservationCount(int cObservations) {// 设置观察值数量
		m_cObservations = cObservations;
	}

	public void addObservation(String sObservation) {// 添加观察值
		if (m_mObservations == null) {
			m_mObservations = new TreeMap<String, Integer>();
		}

		m_mObservations.put(sObservation, m_cObservations);// 将观察值放入m_mObservations中
		m_cObservations++;
	}

	/**
	 * 模拟一个执行：开始状态+动作，按照T Function执行；按概率选出下一个状态
	 * 
	 * @param iAction
	 * @param iState
	 * @return
	 */
	public int execute(int iAction, int iState) {
		int iNextState = -1;
		double dProb = m_rndGenerator.nextDouble();// 随机生成概率
		double dTr = 0.0;
		Iterator<Entry<Integer, Double>> itNonZero = getNonZeroTransitions(
				iState, iAction);// 得到非0转移
		Entry<Integer, Double> e = null;
		while (dProb > 0) {
			e = itNonZero.next();
			iNextState = e.getKey();
			dTr = e.getValue();
			dProb -= dTr;
		}
		return iNextState;
	}

	/**
	 * 模拟得到的观察值
	 * 
	 * @param iAction
	 * @param iState
	 * @return
	 */
	public int observe(int iAction, int iState) {
		int iObservation = -1;
		double dProb = m_rndGenerator.nextDouble(), dO = 0.0;// 随机生成概率
		Iterator<Entry<Integer, Double>> itNonZeroObservations = m_fObservation
				.getNonZeroEntries(iAction, iState);// 得到非0观察
		Entry<Integer, Double> e = null;
		while (dProb > 0) {
			e = itNonZeroObservations.next();
			iObservation = e.getKey();
			dO = e.getValue();
			dProb -= dO;
		}
		if (iObservation == m_cObservations)
			throw new Error("Corrupted observation function - O( "
					+ getActionName(iAction) + ", " + getStateName(iState)
					+ ", * ) = 0");
		return iObservation;
	}

	public double computeAverageDiscountedReward(int cTests,
			int cMaxStepsToGoal, PolicyStrategy policy) {
		return computeAverageDiscountedReward(cTests, cMaxStepsToGoal, policy,
				true);
	}

	/**
	 * 模拟cTests次；每次模拟执行cMaxStepsToGoal步，并计算出本次模拟的折扣回报值；然后输出、返回平均折扣回报值
	 */
	public double computeAverageDiscountedReward(int cTests,
			int cMaxStepsToGoal, PolicyStrategy policy, boolean bOutputMessages) {
		double dSumDiscountedRewards = 0.0, dDiscountedReward = 0.0, dSumSquares = 0.0;
		int iTest = 0; 
		RandomGenerator rndGenerator = m_rndGenerator;// 存储m_rndGenerator
		int[] aiActionCount = new int[m_cActions];// 记录每个动作执行了多少次
		double dStdev = 10000.0, dStandardError = 10.0, dADR = 0.0;// 这里定义是为了满足执行条件

		boolean bCacheBeliefStates = getBeliefStateFactory().cacheBeliefStates(
				false);// 好像是控制多线程的吧

		//m_cSteps = 0;
		// 计算cTests次模拟折扣回报，然后计算平均值
		for (iTest = 0; (iTest < cTests) && (dStandardError > 0.01 * dADR); iTest++) {
			dDiscountedReward = computeDiscountedReward(cMaxStepsToGoal,
					policy, aiActionCount);// 记录计算出本次模拟的折扣回报值
			dSumSquares += (dDiscountedReward * dDiscountedReward);// 记录计算出模拟的回报值平方和
			dSumDiscountedRewards += dDiscountedReward;// 记录计算出模拟的折扣回报值和

			if (iTest >= 50 && iTest % 10 == 0) {
				dADR = dSumDiscountedRewards / iTest;// 平均回报值
				dStdev = Math.sqrt((dSumSquares - (iTest + 1) * dADR * dADR)
						/ iTest);
				if (!Double.isNaN(dStdev))
					dStandardError = 2.0 * dStdev / Math.sqrt(iTest + 1);
			}
		}

		if (bOutputMessages) {
			dADR = dSumDiscountedRewards / iTest;// 平均回报值
			dStdev = Math.sqrt(Math.abs((dSumSquares - (iTest + 1) * dADR
					* dADR)
					/ iTest));// 标准差,这里该加绝对值吧
			dStandardError = 2.0 * dStdev / Math.sqrt(iTest);// 控制条件
			Logger.getInstance().log(
					"POMDP",
					0,
					"computeAverageDiscountedReward",
					"After " + iTest + " tests. ADR " + round(dADR, 3)
							+ ", stdev " + round(dStdev, 5));
			Logger.getInstance().logln();
		}

		m_rndGenerator = rndGenerator;

		getBeliefStateFactory().cacheBeliefStates(bCacheBeliefStates);

		return dSumDiscountedRewards / iTest;
	}

	protected double round(double d, int cDigits) {// 取几位小数点
		double dPower = Math.pow(10, cDigits);
		double d1 = Math.round(d1 = d * dPower);
		return d1 / dPower;
	}

	protected double computeDiscountedReward(int cMaxStepsToGoal,
			PolicyStrategy policy, int[] aiActionCount) {
		return computeDiscountedReward(cMaxStepsToGoal, policy, null, false,
				aiActionCount);
	}

	public double computeDiscountedReward(int cMaxStepsToGoal,
			PolicyStrategy policy, Vector<BeliefState> vObservedBeliefPoints,
			boolean bExplore, int[] aiActionCount) {// 这个方法没什么用
		return computeDiscountedRewardII(cMaxStepsToGoal, policy,
				vObservedBeliefPoints, bExplore, aiActionCount);
	}

	/**
	 * 模拟执行cMaxStepsToGoal步，按指定的策略和起始状态概率及各种函数，计算出折扣回报 R + yR + y2R + y3R + ...
	 * 
	 * @param cMaxStepsToGoal
	 * @param policy
	 * @param vObservedBeliefPoints
	 * @param bExplore
	 * @param aiActionCount
	 *            记录各动作的执行次数
	 * @return
	 */
	public double computeDiscountedRewardII(int cMaxStepsToGoal,
			PolicyStrategy policy, Vector<BeliefState> vObservedBeliefPoints,
			boolean bExplore, int[] aiActionCount) {
		double dDiscountedReward = 0.0, dCurrentReward = 0.0, dDiscountFactor = 1.0;

		int iStep = 0, iAction = 0, iObservation = 0;

		int iState = chooseStartState(), iNextState = 0;// 选择一个初始状态
		BeliefState bsCurrentBelief = getBeliefStateFactory()
				.getInitialBeliefState(), bsNext = null;// 初始化信念点

		boolean bDone = false;
		int cSameStates = 0;

		for (iStep = 0; (iStep < cMaxStepsToGoal) && !bDone; iStep++) {

			if (bExplore) {// 始终为null
				double dRand = m_rndGenerator.nextDouble();
				if (dRand > 0.1)
					iAction = policy.getAction(bsCurrentBelief);
				else
					iAction = m_rndGenerator.nextInt(m_cActions);
			} else {
				iAction = policy.getAction(bsCurrentBelief);// 根据策略和当前信念状态选择一个动作
				if (iAction == -1)
					throw new Error("Could not find optimal action for bs "
							+ bsCurrentBelief);

			}

			if (iAction == -1)
				return Double.NEGATIVE_INFINITY;

			if (aiActionCount != null)
				aiActionCount[iAction]++;// 记录各动作执行了多少次

			if (vObservedBeliefPoints != null) {// 始终为null
				vObservedBeliefPoints.add(bsCurrentBelief);
			}

			// 模拟执行，获得下一个状态
			iNextState = execute(iAction, iState);
			// 模拟出观察值
			iObservation = observe(iAction, iNextState);

			// 获得相应的立即回报
			if (m_rtReward == RewardType.StateAction)
				dCurrentReward = R(iState, iAction); // R(s,a)
			else if (m_rtReward == RewardType.StateActionState)
				dCurrentReward = R(iState, iAction, iNextState); // R(s,a,s')
			else if (m_rtReward == RewardType.State)
				dCurrentReward = R(iState);
			dDiscountedReward += dCurrentReward * dDiscountFactor;
			dDiscountFactor *= m_dGamma;// 折扣

			// 若结束了，还是会继续执行到cMaxStepsToGoal步，但是折扣因子为0
			bDone = endADR(iNextState);// 结束状态
			if (bDone)
				dDiscountFactor = 0.0;

			// 计算出下一个信念点
			bsNext = bsCurrentBelief.nextBeliefState(iAction, iObservation);// 已知下一个状态，更新信念点

			if (iState != iNextState)
				cSameStates = 0;
			else
				cSameStates++;
			if (bsNext == null || (bsNext.valueAt(iNextState) == 0)
					|| (cSameStates > 10)) {
				bDone = true;
			}

			iState = iNextState;
			bsCurrentBelief.release();// 释放当前信念状态，但没实现
			bsCurrentBelief = bsNext;
		}
		return dDiscountedReward;// + m_dMinReward * ( 1 / ( 1 - dDiscountFactor
									// ) );
	}

	/**
	 * 判断是否结束ADR的计算 ADR: average discount reward
	 * 
	 * @param iState
	 * @param dReward
	 * @return
	 */
	public boolean endADR(int iState) {
		return (terminalStatesDefined() && isTerminalState(iState));
	}

	BeliefStateFactory bsf = null;

	/**
	 * 模拟起始状态；按概率取一个开始状态
	 * 
	 * @return
	 */
	public int chooseStartState() {
		int iStartState = -1;
		double dInitialProb = m_rndGenerator.nextDouble();
		double dProb = dInitialProb;
		while (dProb > 0) {
			iStartState++;
			dProb -= probStartState(iStartState);
		}
		return iStartState;
	}

	public boolean terminalStatesDefined() {
		return (m_vTerminalStates != null);
	}

	public boolean isTerminalState(int iState) {
		if (terminalStatesDefined()) {
			return m_vTerminalStates.contains(new Integer(iState));
		}
		return false;
	}

	public int getStateCount() {// 返回状态个数
		return m_cStates;
	}

	public int getActionCount() {
		return m_cActions;
	}

	public int getObservationCount() {
		return m_cObservations;
	}

	public double getDiscountFactor() {
		return m_dGamma;
	}

	public double immediateReward(BeliefState bs, int iAction) {

		if (bs == null)
			return 0.0;

		double dReward = bs.getActionImmediateReward(iAction);
		if (dReward != MIN_INF)
			return dReward;

		dReward = computeImmediateReward(bs, iAction);
		bs.setActionImmediateReward(iAction, dReward);

		return dReward;
	}

	protected double computeImmediateReward(BeliefState bs, int iAction) {
		int iState = 0;
		double dReward = 0.0, dPr = 0.0, dValue = 0.0;

		for (Entry<Integer, Double> e : bs.getNonZeroEntries()) {
			iState = e.getKey();
			dPr = e.getValue();
			dValue = R(iState, iAction);
			dReward += dPr * dValue;
		}
		return dReward;
	}

	/**
	 * 获得和开始状态和动作有关的概率非0的转换
	 * 
	 * @param iStartState
	 * @param iAction
	 * @return
	 */
	public Iterator<Entry<Integer, Double>> getNonZeroTransitions(
			int iStartState, int iAction) {
		return m_fTransition.getNonZeroEntries(iStartState, iAction);
	}

	public Iterator<Entry<Integer, Double>> getNonZeroObservations(int iAction,
			int iEndState) {
		return m_fObservation.getNonZeroEntries(iAction, iEndState);
	}

	public double probStartState(int iState) {
		return m_fStartState.valueAt(iState);// 返回起始概率
	}

	public double getMinR() {
		return m_fReward.getMinValue();
	}

	public double getMaxR() {
		return m_fReward.getMaxValue();
	}

	public double getMaxMinR() {
		int iAction = 0;
		double dMaxR = MIN_INF;
		for (iAction = 0; iAction < m_cActions; iAction++) {
			if (m_adMinActionRewards[iAction] > dMaxR)
				dMaxR = m_adMinActionRewards[iAction];
		}
		return dMaxR;
	}

	public int getStartStateCount() {
		return m_fStartState.countNonZeroEntries();
	}

	@SuppressWarnings("unchecked")
	public Iterator<Entry<Integer, Double>> getStartStates() {
		return m_fStartState.getNonZeroEntries();
	}

	public void initRandomGenerator(long iSeed) {
		m_rndGenerator.init(iSeed);
	}

	public void setRandomSeed(long iSeed) {
		m_iRandomSeed = iSeed;
	}

	public boolean isValid(int iState) {
		return true;
	}

	public Collection<Integer> getValidStates() {
		return new IntegerCollection(0, m_cStates);
	}

	public static class IntegerCollection extends AbstractCollection<Integer>
			implements Serializable {
		private static final long serialVersionUID = 1L;
		
		private int m_iFirst, m_iLast;

		public IntegerCollection(int iFirst, int iLast) {
			m_iFirst = iFirst;
			m_iLast = iLast;
		}

		public Iterator<Integer> iterator() {
			return new IntegerIterator(m_iFirst, m_iLast);
		}

		public int size() {
			return m_iLast - m_iFirst;
		}
	}

	private static class IntegerIterator implements Iterator<Integer> {

		private int m_iNext, m_iLast;

		public IntegerIterator(int iFirst, int iLast) {
			m_iNext = iFirst;
			m_iLast = iLast;
		}

		public boolean hasNext() {
			return m_iNext < m_iLast;
		}

		public Integer next() {
			m_iNext++;
			return m_iNext - 1;
		}

		public void remove() {
		}

	}

	public boolean isFactored() {
		return false;
	}

	public String getName() {
		return m_sName;
	}

	public Vector<Integer> getObservationRelevantStates() {
		return m_vObservationStates;
	}

	public RewardType getRewardType() {
		return m_rtReward;
	}

	public void initBeliefStateFactory() {
		m_bsFactory = new BeliefStateFactory(this, 20);// 实例化信念工厂
	}

	public Collection<Integer> getRelevantActions(BeliefState bs) {
		return new IntegerCollection(0, getActionCount());
	}

	public void addTerminalState(int iTerminalState) {// 添加终止状态
		if (m_vTerminalStates == null)
			m_vTerminalStates = new Vector<Integer>();
		m_vTerminalStates.add(iTerminalState);// 将终止状态放入m_vTerminalStates中
	}

	public void addObservationSensitiveState(int iObservationState) {
		m_vObservationStates.add(iObservationState);// 设置m_vObservationStates
	}

	public void setStartStateProb(int iStartState, double dValue) {
		m_fStartState.setValue(iStartState, dValue);// 设置m_fStartState
	}

	public void setRewardType(RewardType type) {
		m_rtReward = type;

	}

	public void setReward(int iStartState, double dValue) {/* R(s) */
		m_fReward.setValue(iStartState, dValue);

	}

	public AlphaVector newAlphaVector() {
		return new TabularAlphaVector(null, 0, this);
	}

	public void setReward(int iStartState, int iAction, double dValue) {/*
																		 * R(s,a)
																		 */
		m_fReward.setValue(iStartState, iAction, dValue);

	}

	public void setReward(int iStartState, int iAction, int iEndState,
			double dValue) {/* R(s,a,s') */
		m_fReward.setValue(iStartState, iAction, iEndState, dValue);

	}

	public void setMinimalReward(int iAction, double dValue) {// 设置动作最小回报值和最小回报值
		if (iAction != -1) {
			if (dValue < m_adMinActionRewards[iAction]) {
				m_adMinActionRewards[iAction] = dValue;
			}
		}
		if (dValue < m_dMinReward)
			m_dMinReward = dValue;
	}

	/**
	 * 根据状态、动作、观察值的数量初始化各“函数” 主要是初始化“函数类”里面的数据结构的长度
	 */
	public void initDynamicsFunctions() {
		int[] aDims = new int[3];
		aDims[0] = m_cStates;
		aDims[1] = m_cActions;
		aDims[2] = m_cStates;
		m_fTransition = new SparseTabularFunction(aDims);// 实例化状态转移函数，三维
		aDims[0] = m_cActions;
		aDims[1] = m_cStates;
		aDims[2] = m_cObservations;
		m_fObservation = new SparseTabularFunction(aDims);// 实例化观察函数，三维
		aDims = new int[1];
		aDims[0] = m_cStates;
		if (m_cStates > g_sMaxTabularSize)// 实例化初始状态函数，一维
			m_fStartState = new SparseTabularFunction(aDims);
		else
			m_fStartState = new TabularFunction(aDims);
		aDims = new int[3];
		aDims[0] = m_cStates;
		aDims[1] = m_cActions;
		aDims[2] = m_cStates;
		if (m_cStates > g_sMaxTabularSize)// 实例化回报函数，三维
			m_fReward = new SparseTabularFunction(aDims);
		else
			m_fReward = new TabularFunction(aDims);

		m_adMinActionRewards = new double[m_cActions];// 动作的最小回报值函数，一维
		for (int idx = 0; idx < m_cActions; idx++) {
			m_adMinActionRewards[idx] = 0;
		}

	}

	public RandomGenerator getRandomGenerator() {
		return m_rndGenerator;
	}

}
