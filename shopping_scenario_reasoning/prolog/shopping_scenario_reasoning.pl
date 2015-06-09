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
      item_urdf_path/2,
      item_class_type/2,
      object_position/4,
      object_literal_atom/3,
      rr_call/3,
      object_dimensions_restricted/4,
      object_primitive_shape/2
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
    item_urdf_path(r, r),
    item_class_type(r, r),
    object_position(r, r),
    object_literal_atom(r, r, r),
    rr_call(r, r, r),
    object_dimensions_restricted(r, r, r, r),
    object_primitive_shape(r, r).


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
    
    rr_call('resolveRelativePath', [URDFRelative], URDFPath).


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
    
    rdf_triple(knowrob:'level', RackLevel, LevelLiteral),
    strip_literal_type(LevelLiteral, LevelLiteralAtom),
    term_to_atom(Level, LevelLiteralAtom).


%% rr_call(?Function, ?Parameters, ?Result) is nondet.
%
% Calls a function in the Java class 'RackReasoner'. This is a shorthand for jpl_new... and jpl_call... specific to RackReasoner.
%
% @param Function     The function to call
% @param Parameters   The parameters to pass to the function
% @param Result       The returned result
%
rr_call(Function, Parameters, Result) :-
    jpl_new('org.knowrob.shopping_scenario_reasoning.RackReasoner', [], RR),
    jpl_call(RR, Function, Parameters, Result).


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
    object_position(RackLevel, RLX, RLY, RLZ),
    
    object_dimensions_restricted(RackLevel, LevelDepth, LevelWidth, LevelHeight),
    
    rr_call('positionOnRackLevel', [X, Y, Z, RLX, RLY, RLZ, LevelWidth, LevelDepth, LevelHeight], Result),
    jpl_is_true(Result).


%% object_position(?Object, ?X, ?Y, ?Z) is nondet.
%
%  Return the position of the given object.
%
% @param Object       The object to return the position for
% @param X            The X coordinate for the object
% @param Y            The Y coordinate for the object
% @param Z            The Z coordinate for the object
%
object_position(Object, X, Y, Z) :-
    current_object_pose(Object, [_, _, _, X, _, _, _, Y, _, _, _, Z, _, _, _, _]).


%% rack_level_elevation(?RackLevel, ?Elevation) is nondet.
%
%  Returns the elevation (z coordinate) of the given racklevel.
%
% @param RackLevel    The level of the rack to return the elevation of
% @param Elevation    Z coordinate of the rack level (on its surface)
%
rack_level_elevation(RackLevel, Elevation) :-
    object_position(RackLevel, _, _, RLZ),
    object_dimensions_restricted(RackLevel, _, _, LevelHeight),
    
    rr_call('rackLevelElevation', [RLZ, LevelHeight], Elevation).


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
    rack_level_elevation(RackLevel, Elevation),
    object_position(RackLevel, RLX, RLY, _),
    
    rr_call('rackLevelRelativePosition', [RLX, RLY, Elevation, RelativeX, RelativeY], AbsolutePositionArray),
    jpl_array_to_list(AbsolutePositionArray, AbsolutePosition).


%% object_literal_atom(?Object, ?Property, ?Value) is nondet.
%
% Returns the given property (wrapped in a literal and represented as an atom) for the given object.
% @param Object       Object to return the property for
% @param Property     The property to return the literal wrapped/atom represented value for
% @param Value        The contained value
%
object_literal_atom(Object, Property, Value) :-
    owl_has(Object, Property, Literal),
    strip_literal_type(Literal, Atom),
    term_to_atom(Value, Atom).


%% item_class_type(?ClassType, ?Item) is nondet.
%
%  Returns items that are instances of ClassType
%
% @param ClassType    The class type to return items for
% @param Item         Items matching ClassType
%
item_class_type(ClassType, Item) :-
    rdfs_instance_of(Item, ClassType).


%% object_dimensions_restricted(?Object, ?Width, ?Depth, ?Height) is nondet.
%
% Gets the width, depth, and height of a given object, and can traverse into class restrictions.
%
% @param Object       The object to get the dimensions for
% @param Width        Width of the object
% @param Depth        Depth of the object
% @param Height       Height of the object
%
object_dimensions_restricted(Object, Width, Depth, Height) :-
    object_literal_atom(Object, knowrob:'widthOfObject', Width),
    object_literal_atom(Object, knowrob:'depthOfObject', Depth),
    object_literal_atom(Object, knowrob:'heightOfObject', Height).


%% object_primitive_shape(?Object, ?PrimitiveShape) is nondet.
%
% Returns the primitive shape of Object.
%
% @param Object          The object to get the primitive shape of
% @param PrimitiveShape  The primitive shape of Object
%
object_primitive_shape(Object, PrimitiveShape) :-
    object_literal_atom(Object, knowrob:'primitiveShape', PrimitiveShape).
