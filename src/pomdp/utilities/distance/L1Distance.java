package pomdp.utilities.distance;

/**
 * 计算1范数的辅助类
 */
public class L1Distance extends LDistance
{
    protected static L1Distance m_l1Distance;
	
	protected L1Distance()
	{
		super();
	}
	
	public static DistanceMetric getInstance()
	{
		if( m_l1Distance == null )
		{
			m_l1Distance = new L1Distance();
		}
		return m_l1Distance;
	}

	/**
	 * 获得累计值
	 * @param dAccumulated 之前的累计值
	 * @param dValue1 值1
	 * @param dValue2 值2
	 * @return dAccumulated + |dValue1 - dValue2|
	 */
	protected double applyDistanceMetric(double dAccumulated, double dValue1, double dValue2) 
	{
		return dAccumulated + Math.abs( dValue1 - dValue2 );
	}

	protected double applyFinal(double dAccumulated) 
	{
		return dAccumulated;
	}
    
}
