/*
	Development under MIT License.

	Created by: Tyler Fedrizzi
	Created On: 21 March 2020

	Purpose: To calcuate the great circle distance between 2 latitude and longitude points.
	Based upon a spherical Earth model, which can induce up to 0.3% error in measurements.
	Error is accepted for now.
*/

#ifndef msr_airlib_Haversine_hpp
#define msr_airlib_Haversine_hpp

#include<cmath>

namespace msr {
	namespace airlib {
		static double haversine(double lat1, double lon1, double lat2, double lon2)
		{
			// distance between latitudes 
		// and longitudes 
			double dLat = (lat2 - lat1) * M_PI / 180.0;
			double dLon = (lon2 - lon1) * M_PI / 180.0;

			// convert to radians 
			lat1 = (lat1)* M_PI / 180.0;
			lat2 = (lat2)* M_PI / 180.0;

			// apply formulae 
			double a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(lat1) * cos(lat2);
			double rad = 6371; // [km]
			double c = 2 * asin(sqrt(a));
			return rad * c; // distance in [km]
		} // haversine
	}
}
#endif