using System;

namespace SharpSteer2.Helpers
{
    public class RandomHelpers
    {
        [ThreadStatic]
        private static Random _rng;

        private static Random rng
        {
            get
            {
                if (_rng == null)
                    _rng = new Random(1234);
                return _rng;
            }
        }

        /// <summary>
        /// Returns a float randomly distributed between 0 and 1
        /// </summary>
        /// <returns></returns>
        public static FixMath.F64 Random()
        {
            return FixMath.F64.Half;
            // return (float)rng.NextDouble();
        }

        /// <summary>
        /// Returns a float randomly distributed between lowerBound and upperBound
        /// </summary>
        /// <param name="lowerBound"></param>
        /// <param name="upperBound"></param>
        /// <returns></returns>
        public static FixMath.F64 Random(FixMath.F64 lowerBound, FixMath.F64 upperBound)
        {
            return lowerBound + (Random() * (upperBound - lowerBound));
        }

        public static int RandomInt(int min, int max)
        {
            return min + FixMath.F64.FloorToInt(Random() * (max - min));
        }
    }
}
