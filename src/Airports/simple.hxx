// simple.hxx -- a really simplistic class to manage airport ID,
//                 lat, lon of the center of one of it's runways, and
//                 elevation in feet.
//
// Written by Curtis Olson, started April 1998.
// Updated by Durk Talsma, started December 2004.
//
// Copyright (C) 1998  Curtis L. Olson  - http://www.flightgear.org/~curt
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
//
// $Id$


#ifndef _FG_SIMPLE_HXX
#define _FG_SIMPLE_HXX

#include <simgear/compiler.h>

#include <string>
#include <vector>

#include "Navaids/positioned.hxx"

// forward decls
class FGAirportDynamics;
class FGRunway;
class FGTaxiway;

typedef SGSharedPtr<FGRunway> FGRunwayPtr;
typedef SGSharedPtr<FGTaxiway> FGTaxiwayPtr;

/***************************************************************************************
 *
 **************************************************************************************/
class FGAirport : public FGPositioned
{
private:
    SGGeod _tower_location;
    std::string _name;
    bool _has_metar;
    FGAirportDynamics *_dynamics;

public:
    FGAirport(const std::string& id, const SGGeod& location, const SGGeod& tower, 
            const std::string& name, bool has_metar, Type aType);
    ~FGAirport();

    const std::string& getId() const { return ident(); }
    const std::string& getName() const { return _name; }
    double getLongitude() const { return longitude(); }
    // Returns degrees
    double getLatitude()  const { return latitude(); }
    // Returns ft
    double getElevation() const { return elevation(); }
    bool   getMetar()     const { return _has_metar; }
    bool   isAirport()    const;
    bool   isSeaport()    const;
    bool   isHeliport()   const;

    virtual const std::string& name() const
    { return _name; }

    const SGGeod& getTowerLocation() const { return _tower_location; }

    void setMetar(bool value) { _has_metar = value; }

    FGRunway* getActiveRunwayForUsage() const;

    FGAirportDynamics *getDynamics();
    
    unsigned int numRunways() const;
    FGRunway* getRunwayByIndex(unsigned int aIndex) const;

    bool hasRunwayWithIdent(const std::string& aIdent) const;
    FGRunway* getRunwayByIdent(const std::string& aIdent) const;
    FGRunway* findBestRunwayForHeading(double aHeading) const;
    
     /**
     * Useful predicate for FMS/GPS/NAV displays and similar - check if this
     * aiport has a hard-surfaced runway of at least the specified length.
     */
    bool hasHardRunwayOfLengthFt(double aLengthFt) const;
    
    unsigned int numTaxiways() const;
    FGTaxiway* getTaxiwayByIndex(unsigned int aIndex) const;
    
    void setRunwaysAndTaxiways(std::vector<FGRunwayPtr>& rwys,
      std::vector<FGTaxiwayPtr>& txwys);
    
    class AirportFilter : public Filter
     {
     public:
       virtual bool pass(FGPositioned* aPos) const { 
         return passAirport(static_cast<FGAirport*>(aPos));
       }
       
       virtual Type minType() const {
         return AIRPORT;
       }
       
       virtual Type maxType() const {
         return SEAPORT;
       }
       
       virtual bool passAirport(FGAirport* aApt) const {
         return true;
       }
     };
     
     class HardSurfaceFilter : public AirportFilter
     {
     public:
       HardSurfaceFilter(double minLengthFt);
       
       virtual bool passAirport(FGAirport* aApt) const;
       
       virtual Type maxType() const {
         return AIRPORT;
       }
     private:
       double mMinLengthFt;
     };
     
     /**
      * Syntactic wrapper around FGPositioned::findClosest - find the closest
      * match for filter, and return it cast to FGAirport. The default filter
      * passes all airports, including seaports and heliports.
      */
     static FGAirport* findClosest(const SGGeod& aPos, double aCuttofNm, Filter* filter = NULL);
     
     /**
      * Helper to look up an FGAirport instance by unique ident. Throws an 
      * exception if the airport could not be found - so callers can assume
      * the result is non-NULL.
      */
     static FGAirport* getByIdent(const std::string& aIdent);
     
     /**
      * Helper to look up an FGAirport instance by unique ident. Returns NULL
      * if the airport could not be found.
      */
     static FGAirport* findByIdent(const std::string& aIdent);
     
     /**
      * Specialised helper to implement the AirportList dialog. Performs a
      * case-insensitive search on airport names and ICAO codes, and returns
      * matches in a format suitable for use by a puaList. 
      */
     static char** searchNamesAndIdents(const std::string& aFilter);
private:
    typedef std::vector<FGRunwayPtr>::const_iterator Runway_iterator;
    /**
     * Helper to locate a runway by ident
     */
    Runway_iterator getIteratorForRunwayIdent(const std::string& aIdent) const;

    FGAirport operator=(FGAirport &other);
    FGAirport(const FGAirport&);
    
    std::vector<FGRunwayPtr> mRunways;
    std::vector<FGTaxiwayPtr> mTaxiways;
};

// find basic airport location info from airport database
const FGAirport *fgFindAirportID( const std::string& id);

// get airport elevation
double fgGetAirportElev( const std::string& id );

#endif // _FG_SIMPLE_HXX

