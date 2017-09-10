#ifndef MIA_COLOR_H
#define MIA_COLOR_H

//#include <iostream>

namespace mia {
struct Color
{
    short red, green, blue, alfa;

    /// Constructor :
    ///--------------
    inline Color() {
        red= 0;
        green= 0;
        green= 0;
        alfa= 255;
    };
    inline Color(short r, short g, short b, short a= 255) {
        red= r & 0x00FF;
        green= g & 0x00FF;
        blue= b & 0x00FF;
        alfa= a & 0x00FF;
    };

    inline Color(int i) {
        red= short(( i & 0xFF000000 ) >> 24 );
        green= short(( i & 0x00FF0000 ) >> 16 );
        blue= short(( i & 0x0000FF00 ) >> 8 );
        alfa= short( i & 0x000000FF );
    };

    /// Asignement :
    ///-------------
    inline Color &operator=(const Color &c)
    {
        red= c.red;
        green= c.green;
        blue= c.blue;
        alfa= c.alfa;
        return (*this);
    }
    
    inline Color &operator=(const int &i)
    {
        red= short(( i & 0xFF000000 ) >> 24 );
        green= short(( i & 0x00FF0000 ) >> 16 );
        blue= short(( i & 0x0000FF00 ) >> 8 );
        alfa= short( i & 0x000000FF );
        return (*this);
    }

    /// Comparaison :
    ///--------------
    inline bool operator==(const Color &c)const {
        return ( red == c.red && green == c.green && blue == c.blue && alfa == c.alfa );
    }

    /// Scalar mult :
    ///--------------
    /*
    	Color &operator*=(const float &f)
    	{
    		red= (short)(red*f) & 0x00FF;
    		green= (short)(green*f) & 0x00FF;
    		blue= (short)(blue*f) & 0x00FF;
    		alfa= (short)(alfa*f) & 0x00FF;
    		return (*this);
    	}
    	Color &operator/=(const float &f)
    	{
    		float nf= 1/f;
    		red= (short)(red*nf) & 0x00FF;
    		green= (short)(green*nf) & 0x00FF;
    		blue= (short)(blue*nf) & 0x00FF;
    		alfa= (short)(alfa*nf) & 0x00FF;
    		return (*this);
    	}
    	void setAlfa(const float &f){ alfa= (short)(alfa*f) & 0x00FF; }
    */

/// Alfa :
///-------
//  void alterAlfa(float f){ alfa= (short)mia::bound(0.0f, (float)alfa * f, 255.0f); };
//  Color operator*(const float f)const{ return Color(red, green, blue, (short)mia::bound(0.0f, (float)alfa * f, 255.0f) ); };

    /// Blend :
    ///--------
    /*
    	Color &blend(const Color &c)
    	{
    		red= (red >> 1) + ( c.red >> 1 );
    		green= (green >> 1) + ( c.green >> 1 );
    		blue= (blue >> 1) + ( c.blue >> 1 );
    		alfa= (alfa >> 1) + ( c.alfa >> 1 );
    		return this;
    	}
    	Color &blend(const Color &c; float coef)
    	{
    		red*= (red >> 1) + ( c.red >> 1 );
    		green= (green >> 1) + ( c.green >> 1 );
    		blue= (blue >> 1) + ( c.blue >> 1 );
    		alfa= (alfa >> 1) + ( c.alfa >> 1 );
    		return this;
    	}
    */

};

};

#endif // COLOR_H
