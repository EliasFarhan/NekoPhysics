namespace Neko
{
public struct Vec2f
{
    public Fixed x;
    public Fixed y;
    
    public static Vec2f zero = new Vec2f(new Fixed(0), new Fixed(0));

    public Vec2f(Fixed x, Fixed y)
    {
        this.x = x;
        this.y = y;
    }
}
}