/**  <module> shopping_scenario_reasoning

  Copyright (C) 2015 Jan Winkler
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the Institute for Artificial Intelligence,
        University of Bremen nor the names of its contributors may be used to
        endorse or promote products derived from this software without specific
        prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE INSTITUTE FOR ARTIFICIAL INTELLIGENCE BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

  @author Jan Winkler
  @license BSD
*/

:- module(shopping_scenario_reasoning,
    [
      shopping_item/1,
      is_stackable/1,
      rack/1,
      rack_level/2,
      rack_on_level/3,
      position_on_rack/6,
      rack_level_elevation/2,
      rack_level_relative_position/4,
      item_urdf_path/2
    ]).


:- use_module(library('semweb/rdfs')).
:- use_module(library('owl_parser')).
:- use_module(library('owl')).
:- use_module(library('rdfs_computable')).
:- use_module(library('knowrob_owl')).


:-  rdf_meta
    shopping_item(r),
    is_stackable(r, r),
    rack(r),
    rack_level(r, r),
    rack_on_level(r, r, r),
    position_on_rack(r, r, r, r, r, r),
    rack_level_elevation(r, r),
    rack_level_relative_position(r, r, r, r),
    item_urdf_path(r, r).


:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).


%% shopping_item(?Item) is nondet.
%
%  List all objects that are shopping items.
%
% @param Item      Shopping item identifier
%
shopping_item(Item) :-
    rdf_has(Item, rdf:type, A),
    rdf_reachable(A, rdfs:subClassOf, knowrob:'ShoppingItem').


%% item_urdf_path(?Item, ?URDFPath) is nondet.
%
%  Returns the absolute path to the URDF file definition of a given item
%
% @param Item         The item to get the URDF of
% @param URDFPath     The absolute path to the URDF file
%
item_urdf_path(Item, URDFPath) :-
    shopping_item(Item),
    owl_has(Item, knowrob:'urdf', literal(type(_, URDFRelative))),
    
    jpl_new('org.knowrob.shopping_scenario_reasoning.RackReasoner', [], RR),
    jpl_call(RR, 'resolveRelativePath', [URDFRelative], URDFPath).


%% is_stackable(?Item) is nondet.
%
%  Determine whether a given shopping item is stackable, or return all stackable shopping items.
%
% @param Item      Shopping item identifier
%
is_stackable(Item) :-
    shopping_item(Item),
    owl_has(Item, knowrob:'stackable', literal(type(xsd:boolean, true))).


%% rack(?Rack) is nondet.
%
%  List all objects of type 'Rack'.
%
% @param Rack      Rack item identifier
%
rack(Rack) :-
    rdf_has(Rack, rdf:type, A),
    rdf_reachable(A, rdfs:subClassOf, knowrob:'Rack').


%% rack_level(?Rack, ?RackLevel) is nondet.
%
%  List all levels of a rack.
%
% @param Rack         Rack to get the levels from
% @param RackLevel    Rack level of the Rack
%
rack_level(Rack, RackLevel) :-
    rack(Rack),
    rdf_triple(knowrob:'rackLevel', Rack, RackLevel).


%% rack_on_level(?Rack, ?Level, ?RackLevel) is nondet.
%
%  Return racklevel of rack on level ?Level.
%
% @param Rack         Rack to get the level from
% @param Level        Level to acquire
% @param RackLevel    Rack level of the Rack
%
rack_on_level(Rack, Level, RackLevel) :-
    rack(Rack),
    rdf_triple(knowrob:'rackLevel', Rack, RackLevel),
    rdf_triple(knowrob:'level', RackLevel, LevelLiteral), strip_literal_type(LevelLiteral, LevelLiteralAtom), term_to_atom(Level, LevelLiteralAtom).


%% position_on_rack(?X, ?Y, ?Z, ?LevelHeight, ?Rack, ?RackLevel) is nondet.
%
%  Identify the rack and its respective level where the given position is residing, if any.
%
% @param X            The X coordinate for which to identify the rack and level
% @param Y            The Y coordinate for which to identify the rack and level
% @param Z            The Z coordinate for which to identify the rack and level
% @param LevelHeight  The height above a level that still counts towards it
% @param Rack         The rack on which the position is
% @param RackLevel    The level on the identified rack on which the pose resides
%
position_on_rack(X, Y, Z, LevelHeight, Rack, RackLevel) :-
    rack(Rack),
    rack_level(Rack, RackLevel),
    current_object_pose(RackLevel, [_, _, _, RLX, _, _, _, RLY, _, _, _, RLZ, _, _, _, _]),
    
    rdf_has(RackLevel, knowrob:'widthOfObject', WidthLiteral),
    strip_literal_type(WidthLiteral, WidthLiteralAtom),
    term_to_atom(LevelWidth, WidthLiteralAtom),
    
    rdf_has(RackLevel, knowrob:'depthOfObject', DepthLiteral),
    strip_literal_type(DepthLiteral, DepthLiteralAtom),
    term_to_atom(LevelDepth, DepthLiteralAtom),
    
    jpl_new('org.knowrob.shopping_scenario_reasoning.RackReasoner', [], RR),
    jpl_call(RR, 'positionOnRackLevel', [X, Y, Z, RLX, RLY, RLZ, LevelWidth, LevelDepth, LevelHeight], Result),
    jpl_is_true(Result).


%% rack_level_elevation(?RackLevel, ?Elevation) is nondet.
%
%  Returns the elevation (z coordinate) of the given racklevel.
%
% @param RackLevel    The level of the rack to return the elevation of
% @param Elevation    Z coordinate of the rack level (on its surface)
%
rack_level_elevation(RackLevel, Elevation) :-
    current_object_pose(RackLevel, [_, _, _, _, _, _, _, _, _, _, _, RLZ, _, _, _, _]),
    
    rdf_has(RackLevel, knowrob:'heightOfObject', HeightLiteral),
    strip_literal_type(HeightLiteral, HeightLiteralAtom),
    term_to_atom(LevelHeight, HeightLiteralAtom),
    
    jpl_new('org.knowrob.shopping_scenario_reasoning.RackReasoner', [], RR),
    jpl_call(RR, 'rackLevelElevation', [RLZ, LevelHeight], Elevation).


%% rack_level_relative_position(?RackLevel, ?RelativeX, ?RelativeY, ?AbsolutePosition) is nondet.
%
%  Returns the RwlativeX/RelativeY relative position on the rack level ?RackLevel.
%
% @param RackLevel         The level of the rack to return the relative position on
% @param RelativeX         Relative position in X direction
% @param RelativeY         Relative position in Y direction
% @param AbsolutePosition  Absolute position generated
%
rack_level_relative_position(RackLevel, RelativeX, RelativeY, AbsolutePosition) :-
    current_object_pose(RackLevel, [_, _, _, RLX, _, _, _, RLY, _, _, _, RLZ, _, _, _, _]),
    
    rdf_has(RackLevel, knowrob:'heightOfObject', HeightLiteral),
    strip_literal_type(HeightLiteral, HeightLiteralAtom),
    term_to_atom(LevelHeight, HeightLiteralAtom),
    
    jpl_new('org.knowrob.shopping_scenario_reasoning.RackReasoner', [], RR),
    jpl_call(RR, 'rackLevelElevation', [RLZ, LevelHeight], Elevation),
    jpl_call(RR, 'rackLevelRelativePosition', [RLX, RLY, RLZ, RelativeX, RelativeY], AbsolutePositionArray),
    jpl_array_to_list(AbsolutePositionArray, AbsolutePosition).
