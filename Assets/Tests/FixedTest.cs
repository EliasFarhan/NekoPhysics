using System.Collections;
using System.Collections.Generic;
using Neko;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

public class FixedTest
{
    [Test]
    public void EqualFixed()
    {
        {
            Fixed a = new Fixed(1, 0);
            Fixed b = new Fixed(1, 0);
            Assert.True(a == b);
        }
        {
            Fixed a = new Fixed(1, 0);
            Fixed b = new Fixed(-1, 0);
            Assert.False(a == b);
        }
        {
            Fixed a = new Fixed(1, 0);
            Fixed b = new Fixed(2, -1);
            Assert.True(a == b);
        }
        {
            Fixed a = new Fixed(-1, 0);
            Fixed b = new Fixed(-2, -1);
            Assert.True(a == b);
        }
    }
    [Test]
    public void AddFixed()
    {
        {
            Fixed a = new Fixed(1, 0);
            Fixed b = new Fixed(2, 0);
            Assert.True(new Fixed(3, 0) == a + b);
        }
        {
            Fixed a = new Fixed(1, 0);
            Fixed b = new Fixed(2, -1);
            Assert.True(new Fixed(2, 0) == a + b);
        }
    }
    [Test]
    public void SubtractFixed()
    {
        {
            Fixed a = new Fixed(1, 0);
            Fixed b = new Fixed(2, 0);
            Assert.True(new Fixed(1, 0) == b-a);
        }
        {
            Fixed a = new Fixed(1, 0);
            Fixed b = new Fixed(2, -1);
            Assert.True(new Fixed(0, 0) == a - b);
        }
    }

}
