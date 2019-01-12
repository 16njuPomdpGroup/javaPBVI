package pomdp.utilities;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.Vector;

public class BeliefStateVector<E> extends Vector<E> 
{
	private static final long serialVersionUID = 1L;
    
	/**
	 * 0：根    1：第一层    2：第二层
	 */
	private ArrayList<ArrayList<E>> treeLevelInfo = null;
	
	/**
	 * 如果只存放新的b，那么完整的tree信息，就放在这
	 */
	private ArrayList<ArrayList<E>> treeLevelInfoComplete = null;
	
	private boolean notComplete = false;
	
	public BeliefStateVector()
	{
		super();
		treeLevelInfo = new ArrayList<ArrayList<E>>();
	}
	
	public BeliefStateVector(Vector<E> vector)
	{
		super(vector);
	}
	
	public BeliefStateVector(BeliefStateVector<E> beliefStateVector) 
	{
		super(beliefStateVector);
		ArrayList<ArrayList<E>> treeLevelInfoIn = beliefStateVector.getTreeLevelInfo();
		treeLevelInfo = new ArrayList<ArrayList<E>>();
		for(int i = 0; i < treeLevelInfoIn.size(); i++)
		{
			treeLevelInfo.add(new ArrayList<E>());
			treeLevelInfo.get(i).addAll(treeLevelInfoIn.get(i));
		}
	}
	
	public ArrayList<ArrayList<E>> getTreeLevelInfo()
	{
		return treeLevelInfo;
	}
	
	public synchronized void add(E parent, E current)
	{
		super.add(current);
		//为根
		if(parent==null)
		{
			treeLevelInfo.clear();
			ArrayList<E> levelOne = new ArrayList<E>();
			treeLevelInfo.add(levelOne);
			levelOne.add(current);
		}
		else//非根
		{
			int pLevelNum = getLevelNum(parent);
			if(pLevelNum<0)//parent不存在,这样无法添加
			{
				Logger.getInstance().logln("Error: BeliefStateVector.add(E parent, E current): parent not exist!");
		        return;
			}
			else// parent正常
			{
				pLevelNum += 1;
				if(treeLevelInfo.size()<=pLevelNum)//需要新增一层
				{
					ArrayList<E> newLevel = new ArrayList<E>();
					treeLevelInfo.add(newLevel);
					if(notComplete)
					{
						ArrayList<E> newLevel2 = new ArrayList<E>();
						treeLevelInfoComplete.add(newLevel2);
					}
				}
				treeLevelInfo.get(pLevelNum).add(current);//找到层再添加
				if(notComplete)
				{
					treeLevelInfoComplete.get(pLevelNum).add(current);
				}
			}
		}
	}
	
	/**
	 * 查询一个b的层数
	 * 
	 * @param e
	 * @return -1: not found
	 */
	public synchronized int getLevelNum(E e)
	{
		if(notComplete)
		{
			for(int i=0;i<treeLevelInfoComplete.size();i++)
			{
				if(treeLevelInfoComplete.get(i).contains(e))
				{
					return i;
				}
			}
		}
		else
		{
			for(int i=0;i<treeLevelInfo.size();i++)
			{
				if(treeLevelInfo.get(i).contains(e))
				{
					return i;
				}
			}
		}
		return -1;
	}
	
	/**
	 * 获得一个在tree上，从下往上扫描的iterator
	 * 
	 * @return
	 */
	public Iterator<E> getTreeDownUpIterator() 
	{
		ArrayList<E> resultList = new ArrayList<E>();
		for (int i = treeLevelInfo.size() - 1; i >= 0; i--) 
		{
			resultList.addAll(treeLevelInfo.get(i));//addAll(E),这里E是一个集合,那么会将E中所有的元素都加入list中
		}
		return resultList.iterator();
	}
}














