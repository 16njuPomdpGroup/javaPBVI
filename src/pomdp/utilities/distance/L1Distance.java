package pomdp.utilities.distance;

/**
 * ����1�����ĸ�����
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
	 * ����ۼ�ֵ
	 * @param dAccumulated ֮ǰ���ۼ�ֵ
	 * @param dValue1 ֵ1
	 * @param dValue2 ֵ2
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
