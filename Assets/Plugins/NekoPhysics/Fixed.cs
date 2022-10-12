

using Codice.Client.GameUI.Checkin;

namespace Neko
{
    public struct Fixed
    {
        private int value;
        //2^scale scaling
        private int scale;

        public Fixed(int value)
        {
            this.value = value;
            scale = 0;
        }

        public Fixed(int value, int scale)
        {
            this.value = value;
            this.scale = scale;
        }

        public void SetNewScale(int newScale)
        {
            int deltaScale = scale - newScale;
            if (value >= 0)
            {
                value <<= deltaScale;
            }
            else
            {
                int tmp = -value;
                tmp <<= deltaScale;
                value = -tmp;
            }
        }

        public static Fixed operator +(Fixed f) => f;
        public static Fixed operator -(Fixed f) => new Fixed(-f.value, f.scale);

        public static Fixed operator +(Fixed a, Fixed b)
        {
            if (a.scale == b.scale)
            {
                return new Fixed(a.value + b.value, a.scale);
            }
            int commonScale = a.scale < b.scale ? a.scale : b.scale;
            if (a.scale != commonScale)
            {
                a.SetNewScale(commonScale);
            }

            if (b.scale != commonScale)
            {
                b.SetNewScale(commonScale);
            }
            
            return new Fixed(a.value+b.value, commonScale);
        }

        public static Fixed operator -(Fixed a, Fixed b) => a + (-b);

        public static Fixed operator *(Fixed a, Fixed b) => new Fixed(a.value * b.value, a.scale + b.scale);

        public static bool operator ==(Fixed a, Fixed b)
        {
            if (a.value == b.value && a.scale == b.scale) return true;
            int commonScale = a.scale < b.scale ? a.scale : b.scale;
            if (a.scale != commonScale)
            {
                a.SetNewScale(commonScale);
            }

            if (b.scale != commonScale)
            {
                b.SetNewScale(commonScale);
            }
            return a.value == b.value;
        }

        public static bool operator !=(Fixed a, Fixed b)
        {
            return !(a == b);
        }
    }
   
}