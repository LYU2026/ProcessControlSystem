function u = typeN( t )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
c_i=[0.000000000000E+00,0.261591059620E-01,0.109574842280E-04,-0.938411115540E-07,-0.464120397590E-10,-0.263033577160E-11, -0.226534380030E-13,-0.760893007910E-16,-0.934196678350E-19];
c_i1=[0.000000000000E+00,0.259293946010E-01,0.157101418800E-04,0.438256272370E-07,-0.252611697940E-09,0.643118193390E-12,-0.100634715190E-14, 0.997453389920E-18,-0.608632456070E-21,0.208492293390E-24,-0.306821961510E-28];
u=0;
if(t<0)
     for i=0:8
         u=u+c_i(i+1).*(t.^i);
     end;
else
     for i=0:10
        u=u+c_i1(i+1).*(t.^i);
     end;
end

