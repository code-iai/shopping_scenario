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
      test_rack_reasoner/1
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
    test_rack_reasoner(r).

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


%% is_stackable(?Item) is nondet.
%
%  Determine whether a given shopping item is stackable, or return all stackable shopping items.
%
% @param Item      Shopping item identifier
%
is_stackable(Item) :-
    shopping_item(Item),
    rdf_has(Item, knowrob:'stackable', literal(type(xsd:boolean, true))).


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


%% pose_on_rack(Pose, Rack, RackLevel) is nondet.
%
%  Identify the rack and its respective level where the given pose is residing, if any.
%
% @param Pose         The pose for which to identify the rack and level
% @param Rack         The rack on which the pose is
% @param RackLevel    The level on the identified rack on which the pose resides
pose_on_rack_level(Pose, Rack, RackLevel) :-
    false.


%% test_rack_reasoner(Test) is nondet.
%
%  Test the JPL class RackReasoner.
%
test_rack_reasoner(Test) :-
    jpl_new('org.knowrob.shopping_scenario_reasoning.RackReasoner', [], RR),
    jpl_call(RR, 'test', [], _).
