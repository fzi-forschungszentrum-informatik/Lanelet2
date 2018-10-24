# Maps, Projections and Coordinate Reference Systems (CRSs)

### Introduction

So, we just store .osm files in LatLon (like GPS) and Lanelet2 converts them to UTM (by default UTM32N), right? Why is there a file for explaining that?  
Not so fast, my young boy!

Let's say we stand on this point and want to know where we are. Let's get our GPS device out, measure the point and we know it, right?  
Just for the moment. You're still too fast.

But, I already incorporated ionospheric distortions, multi-path effects and did a long-time measurement!  
Well, you still stand in Europe, but your data might very well live in the center of the earth.

What do you mean?  
Look, your GPS device gives you LatLon in the GPS's WGS84 frame, which is fixed to the earth, but the tectonic plate you're standing on is drifting relative to it with about 2.5 cm per year or more than 0.5 m(!) since 1989.

Ok, so what do you propose?

### Lanelet2's conventions

#### Map Files (.osm)

.osm maps are stored in EPSG:4258, i.e. we use LatLon as usual, but we define them to be coupled to the European Reference Frame ETRF89. The frame was congruent with ITRF (and somehow WGS84), but is drifting away now. As we do not want to shift our maps every few years, we fix them in ETRF which is good enough (roughly <1 cm local deviation).

So, what does that mean for me?  
For editing in JOSM, we can either pick WGS84 as projection or use EPSG:4258 by EPSG code.  
This is due to the facts that  
1.) JOSM doesn't incorporate the tectonic shift between WGS84/ITRF (WGS84 is not supposed to be much more accurate than 1 m, but it is aligned with ITRF at an accuracy of <0.1 m) and ETRF89 and  
2.) the difference between the WGS84 and the GRS80 ellipsoids (GRS80 is the basis for EPSG:4258/ETRF89) is <0.1 mm (even for us that is precise enough).

What do I need to think of when importing public OpenStreetMap data?  
OSM data is by default defined in WGS84, thus you need to incorporate a time-dependent tectonic plate shift. This is supposed to be an option for you in Lanelet2, however you might still need to wiggle around the data a bit as most OpenStreetMap mapper do not care about that 0.5 m difference between ETRF89 and WGS84.

#### Internal Data

Lanelet2 loads and reprojects .osm maps to a UTM, but still leaves them in ETRF89, i.e. we use projection EPSG:25832.  

#### GPS Data

GPS data is in WGS84 and normally temporarily distorted by ionospheric distortions. So, either you compensate both the ionospheric distortions *and* the continental drift using e.g. SAPOS or you still need to correct the time-dependent drift after compensating for the ionospheric distortions.

Martin Lauer already implemented the time-dependent shift, but this you be either an (open) dependency of LLT2 or become part of it (Martin already agreed with it).

### WIP: A short primer on the most common projections

#### Lanelet1 Projection?

What's about the projection used in liblanelet (v1)?  
It is Mercator but with a customly selected reference point. Would have the advantage of backward compatibility.

#### WGS84 (GPS coordinates, lat/lon, ...)

OSM **data** is stored in WGS84 (aka EPSG:4326) as are GPS positions by definition. It assumes the Earth is an ellipsoid (the corrected WGS84/EGM96; it's quite ok) and references the height w.r.t. this ellipsoid as *altitude*. However, as we assume that the map is ground-located, we'll refer to this as *elevation* (as it's done in the [OSM](http://wiki.openstreetmap.org/wiki/Key:ele) [standard](http://wiki.openstreetmap.org/wiki/Altitude)).

#### UTM (east/north)

By default, UTM is the projection which you probably want to use for everything.

Why? It's metric, distances are usually (i.e. within some distance (~180 km) to the reference meridian) okay, i.e. they are at most 0.04 % too short (at the reference meridian) which is 40 cm on 1 km, and the conversion from e.g. WGS84 to UTM is precise up to several nm (yes, nanometers!) if you use Karney's GeographicLib with TransverseMercatorExact as projection.

Due to that reason, we use UTM as projection for internal data representation in Lanelet2 with a default of UTM32N. However, we do not use EPSG:32632 (which is what UTM32N commonly refers to) as one could expect, but EPSG:25832. This means that we stay local to the ETRF89 and thus fixed to the European tectonic plate, but everything else is the same as with the 'normal' UTM32N. The WGS84/GRS80 (EPSG:32632/EPSG:25832) ellipsoid difference is negligible (<0.1 mm).

#### Web Mercator

Google Maps, Bing Maps and OSM *Tiles* are in Web Mercator. This is similar to WGS84, but uses a spherical assumptions. This is **not** supposed to be used in Lanelet2.

### Links

#### Things to know
[A guide to coordinate systems (in great britain)](https://www.ordnancesurvey.co.uk/docs/support/guide-coordinate-systems-great-britain.pdf)  
[Transverse Mercator with an accuracy of a few nanometers](https://link.springer.com/article/10.1007%2Fs00190-011-0445-3) [arXiv Mirror](https://arxiv.org/pdf/1002.1417.pdf)  
[Algorithms for geodesics](https://link.springer.com/article/10.1007%2Fs00190-012-0578-z) [arXiv Mirror](https://arxiv.org/pdf/1109.4448.pdf)  
[Geodesics on an ellipsoid of revolution](https://arxiv.org/pdf/1102.1215.pdf)
[Karney's GeographicLib](https://geographiclib.sourceforge.io/)  
[Vincenty's Formulae](https://en.wikipedia.org/wiki/Vincenty%27s_formulae) (Note: Karney's work and his solution to the inverse Geodesic problem (given two points, what's the distance?) obliterate Vincenty's method. Only the performance should be compared as Vincenty's method is already sufficiently accurate for our use cases.)

#### Height-related stuff
[Sind die Bezugssysteme WGS84 und ETRS89 wirklich gleich?](http://www.killetsoft.de/t_1009_d.htm)  
[A tutorial on Datums](https://vdatum.noaa.gov/docs/datums.html)  
[Elevation in OSM](http://wiki.openstreetmap.org/wiki/Key:ele)  
[Altitude in OSM](http://wiki.openstreetmap.org/wiki/Altitude)  
[Mean Sea Level, GPS, and the Geoid](http://www.esri.com/news/arcuser/0703/geoid1of3.html)  