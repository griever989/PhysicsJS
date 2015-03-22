/**
 * PhysicsJS v0.7.0 - 2015-03-22
 * A modular, extendable, and easy-to-use physics engine for javascript
 * http://wellcaffeinated.net/PhysicsJS
 *
 * Copyright (c) 2015 Jasper Palfree <jasper@wellcaffeinated.net>
 * Licensed MIT
 */
(function (root, factory) {
    if (typeof define === 'function' && define.amd) {
        define(['physicsjs'], factory);
    } else if (typeof exports === 'object') {
        module.exports = factory.apply(root, ['physicsjs'].map(require));
    } else {
        factory.call(root, root.Physics);
    }
}(this, function (Physics) {
    'use strict';
    /**
     * class SweepPruneBehavior < Behavior
     *
     * `Physics.behavior('sweep-prune')`.
     *
     * Sweep and Prune implementation for broad phase collision detection.
     *
     * This massively improves the speed of collision detection. It's set up to always be used with [[BodyCollisionDetection]], and [[BodyImpulseResponse]].
     *
     * Additional options include:
     * - channel: The channel to publish collision candidates to. (default: `collisions:candidates`)
     **/
    Physics.behavior('sweep-prune', function( parent ){
    
        var uid = 1;
    
        // Get a unique numeric id for internal use
        var getUniqueId = function getUniqueId(){
    
            return uid++;
        };
    
        // add z: 2 to get this to work in 3D
        var dof = { x: 0, y: 1 }; // degrees of freedom
        // change to "3" to get it to work in 3D
        var maxDof = 2;
    
        var pairHash = Physics.util.pairHash;
    
        return {
    
            // extended
            init: function( options ){
    
                parent.init.call( this );
                this.options.defaults({
                    channel: 'collisions:candidates' //default channel
                });
                this.options( options );
    
                this.encounters = [];
                this.candidates = [];
    
                this.partitionWidth = 0;
                this.partitionHeight = 0;
    
                this.clear();
            },
    
            /**
             * SweepPruneBehavior#clear()
             *
             * Refresh tracking data
             **/
            clear: function(){
    
                this.tracked = [];
                this.pairs = []; // pairs selected as candidate collisions by broad phase
                this.intervalSections = [[]];
                this.scratchSections = [];
                this.sectionMinX = 0;
                this.sectionMinY = 0;
                this.sectionMaxX = 0;
                this.sectionMaxY = 0;
                this.addIntervalSection( 0, 0 );
            },
    
            addIntervalSection: function ( x, y ){
    
                this.sectionMinX = Math.min(this.sectionMinX, x);
                this.sectionMinY = Math.min(this.sectionMinY, y);
                this.sectionMaxX = Math.max(this.sectionMaxX, x);
                this.sectionMaxY = Math.max(this.sectionMaxY, y);
    
                if (this.intervalSections[x] === undefined) {
                    this.intervalSections[x] = [];
                }
                if (this.intervalSections[x][y] === undefined) {
                    var intervalList = [];
                    for ( var xyz = 0; xyz < maxDof; ++xyz ){
    
                        intervalList[ xyz ] = [];
                    }
    
                    this.intervalSections[x][y] = intervalList;
                }
            },
    
            addIntervalSectionFillEmpty: function ( x, y ){
    
                var i, il, j, jl;
    
                this.sectionMinX = Math.min(this.sectionMinX, x);
                this.sectionMinY = Math.min(this.sectionMinY, y);
                this.sectionMaxX = Math.max(this.sectionMaxX, x);
                this.sectionMaxY = Math.max(this.sectionMaxY, y);
    
                for (i = this.sectionMinX, il = this.sectionMaxX; i <= il; i++) {
                    for (j = this.sectionMinY, jl = this.sectionMaxY; j <= jl; j++) {
                        if (this.intervalSections[i] === undefined) {
                            this.intervalSections[i] = [];
                        }
                        if (this.intervalSections[i][j] === undefined) {
                            var intervalList = [];
                            for ( var xyz = 0; xyz < maxDof; ++xyz ){
    
                                intervalList[ xyz ] = [];
                            }
    
                            this.intervalSections[i][j] = intervalList;
                        }
                    }
                }
            },
    
            findIntersectingSections: function ( min, max, createEmpty ){
                var i, j, xmin, xmax, ymin, ymax, section;
    
                Physics.util.clearArray( this.scratchSections );
    
                if (this.isWorldPartitioned()) {
                    xmin = Math.floor(min.x / this.partitionWidth);
                    ymin = Math.floor(min.y / this.partitionHeight);
                    xmax = Math.floor(max.x / this.partitionWidth);
                    ymax = Math.floor(max.y / this.partitionHeight);
    
                    for (j = ymin; j <= ymax; j++) {
                        for (i = xmin; i <= xmax; i++) {
                            section = this.intervalSections[i] !== undefined ? this.intervalSections[i][j] : undefined;
                            if (section === undefined && createEmpty) {
                                this.addIntervalSectionFillEmpty(i, j);
                                section = this.intervalSections[i][j];
                            }
                            if (section !== undefined) {
                                this.scratchSections.push(section);
                            }
                        }
                    }
                } else {
                    this.scratchSections.push(this.intervalSections[0][0]);
                }
    
                return this.scratchSections;
            },
    
            /**
             * SweepPruneBehavior#setPartitionDimensions( x, y )
             *
             * Setting dimensions to any value other than 0 will cause
             * the interval lists to be split over a set of dimensions
             * each equal to the size set. This can be used to partition
             * the world in to sections and only perform collision detection
             * in specific sections.
             **/
            setPartitionDimensions: function( x, y ){
    
                if ( (x === 0 && y === 0) || (x !== 0 && y !== 0) ) {
                    this.partitionWidth = x;
                    this.partitionHeight = y;
                    if (this.tracked.length > 0) {
                        throw 'Error: Attempt to change dimensions with body data present.';
                    }
                    this.intervalSections = [[]];
                    this.addIntervalSection( 0, 0 );
                } else {
                    throw 'Error: Attempt to set invalid partition dimensions. Both dimensions must be either zero or both non-zero.';
                }
            },
    
            isWorldPartitioned: function (){
                return !(this.partitionWidth === 0 && this.partitionHeight === 0);
            },
    
            // extended
            connect: function( world ){
    
                world.on( 'add:body', this.trackBody, this );
                world.on( 'remove:body', this.untrackBody, this );
                world.on( 'integrate:positions', this.sweep, this, 1 );
    
                // add current bodies
                var bodies = world.getBodies();
                for ( var i = 0, l = bodies.length; i < l; ++i ){
    
                    this.trackBody({ body: bodies[ i ] });
                }
            },
    
            // extended
            disconnect: function( world ){
    
                world.off( 'add:body', this.trackBody, this );
                world.off( 'remove:body', this.untrackBody, this );
                world.off( 'integrate:positions', this.sweep, this, 1 );
                this.clear();
            },
    
            /** internal
             * SweepPruneBehavior#broadPhase() -> Array
             * + (Array): The candidate data of overlapping aabbs
             *
             * Execute the broad phase and get candidate collisions
             **/
            broadPhase: function(){
                this.updateIntervals();
                this.sortIntervalLists();
    
                Physics.util.clearArray( this.candidates );
                var i, l, j, lj;
                for ( i = this.sectionMinX, l = this.sectionMaxX; i <= l; i++ ){
                    for ( j = this.sectionMinY, lj = this.sectionMaxY; j <= lj; j++) {
                        this.checkOverlaps( i, j );
                    }
                }
    
                return this.candidates;
            },
    
            /** internal
             * SweepPruneBehavior#sortIntervalLists()
             *
             * Simple insertion sort for each axis
             **/
            sortIntervalLists: function(){
    
                var list
                    ,section
                    ,len
                    ,i
                    ,hole
                    ,bound
                    ,boundVal
                    ,left
                    ,leftVal
                    ,axis
                    ,sx
                    ,sy
                    ,sxl
                    ,syl
                    ;
    
                for ( sx = this.sectionMinX, sxl = this.sectionMaxX; sx <= sxl; sx++ ){
    
                    for ( sy = this.sectionMinY, syl = this.sectionMaxY; sy <= syl; sy++ ) {
                        section = this.intervalSections[ sx ][ sy ];
    
                        // for each axis...
                        for ( var xyz = 0; xyz < maxDof; ++xyz ){
    
                            // get the intervals for that axis
                            list = section[ xyz ];
                            i = 0;
                            len = list.length;
                            axis = xyz;
    
                            // for each interval bound...
                            while ( (++i) < len ){
    
                                // store bound
                                bound = list[ i ];
                                boundVal = bound.val.get( axis );
                                hole = i;
    
                                left = list[ hole - 1 ];
                                leftVal = left && left.val.get( axis );
    
                                // while others are greater than bound...
                                while (
                                    hole > 0 &&
                                    (
                                        leftVal > boundVal ||
                                        // if it's an equality, only move it over if
                                        // the hole was created by a minimum
                                        // and the previous is a maximum
                                        // so that we detect contacts also
                                        leftVal === boundVal &&
                                        ( left.type && !bound.type )
                                    )
                                ) {
    
                                    // move others greater than bound to the right
                                    list[ hole ] = left;
                                    hole--;
                                    left = list[ hole - 1 ];
                                    leftVal = left && left.val.get( axis );
                                }
    
                                // insert bound in the hole
                                list[ hole ] = bound;
                            }
                        }
                    }
                }
            },
    
            /** internal
             * SweepPruneBehavior#getPair( tr1, tr2, doCreate ) -> Object
             * - tr1 (Object): First tracker
             * - tr2 (Object): Second tracker
             * - doCreate (Boolean): Create if not found
             * + (Object): Pair object or null if not found
             *
             * Get a pair object for the tracker objects
             **/
            getPair: function(tr1, tr2, doCreate){
    
                if ( tr1.body.treatment === 'static' && tr2.body.treatment === 'static' ) {
                    return null;
                }
    
                var hash = pairHash( tr1.id, tr2.id );
    
                if ( hash === false ){
                    return null;
                }
    
                var c = this.pairs[ hash ];
    
                if ( c === undefined ){
    
                    if ( !doCreate ){
                        return null;
                    }
    
                    c = this.pairs[ hash ] = {
                        bodyA: tr1.body,
                        bodyB: tr2.body,
                        flag: 1
                    };
                }
    
                if ( doCreate ){
                    c.flag = 1;
                }
    
                return c;
            },
    
            /** internal
             * SweepPruneBehavior#checkOverlaps( x, y ) -> Array
             * + (Array): List of candidate collisions
             *
             * Check each axis for overlaps of bodies AABBs and adds to the current list of candidates.
             **/
            checkOverlaps: function( x, y ){
    
                var isX
                    ,hash
                    ,tr1
                    ,tr2
                    ,bound
                    ,list
                    ,len
                    ,i
                    ,j
                    ,c
                    // determine which axis is the last we need to check
                    ,collisionFlag = 1 << (dof.z + 1) << (dof.y + 1) << (dof.x + 1)
                    ,encounters = this.encounters
                    ,enclen = 0
                    ,candidates = this.candidates
                    ;
    
                Physics.util.clearArray( encounters );
    
                for ( var xyz = 0; xyz < maxDof; ++xyz ){
    
                    // is the x coord
                    isX = (xyz === 0);
                    // get the interval list for this axis
                    list = this.intervalSections[ x ][ y ][ xyz ];
    
                    // for each interval bound
                    for ( i = 0, len = list.length; i < len; i++ ){
    
                        bound = list[ i ];
                        tr1 = bound.tracker;
    
                        if ( bound.type ){
    
                            // is a max
    
                            j = enclen;
    
                            for ( j = enclen - 1; j >= 0; j-- ){
    
                                tr2 = encounters[ j ];
    
                                // if they are the same tracked interval
                                if ( tr2 === tr1 ){
    
                                    // remove the interval from the encounters list
                                    // faster than .splice()
                                    if ( j < enclen - 1 ) {
    
                                        encounters[ j ] = encounters.pop();
    
                                    } else {
    
                                        // encountered a max right after a min... no overlap
                                        encounters.pop();
                                    }
    
                                    enclen--;
    
                                } else {
    
                                    // check if we have flagged this pair before
                                    // if it's the x axis, create a pair
                                    c = this.getPair( tr1, tr2, isX );
    
                                    if ( c && c.flag < collisionFlag ){
    
                                        // if it's greater than the axis index, set the flag
                                        // to = 0.
                                        // if not, increment the flag by one.
                                        c.flag = c.flag << (xyz + 1);
    
                                        // c.flag will equal collisionFlag
                                        // if we've incremented the flag
                                        // enough that all axes are overlapping
                                        if ( c.flag === collisionFlag ){
    
                                            // overlaps on all axes.
                                            // add it to possible collision
                                            // candidates list for narrow phase
    
                                            candidates.push( c );
                                        }
                                    }
                                }
                            }
    
                        } else {
    
                            // is a min
                            // just add this minimum to the encounters list
                            enclen = encounters.push( tr1 );
                        }
                    }
                }
    
                return candidates;
            },
    
            /** internal
             * SweepPruneBehavior#updateIntervals()
             *
             * Update position intervals on each axis
             **/
            updateIntervals: function(){
    
                var tr
                    ,intr
                    ,aabb
                    ,list = this.tracked
                    ,i = list.length
                    ,crossedBoundary = false
                    ,sections
                    ,sectionsQuery
                    ,sc
                    ,sl
                    ;
    
                // for all tracked bodies
                while ( (--i) >= 0 ){
    
                    tr = list[ i ];
                    intr = tr.interval;
                    sections = tr.sections;
                    aabb = tr.body.aabb();
    
                    if ( this.isWorldPartitioned() ){
                        if (
                            (Math.floor(intr.min.val.x / this.partitionWidth) !== Math.floor((aabb.x - aabb.hw) / this.partitionWidth)) ||
                            (Math.floor(intr.max.val.x / this.partitionWidth) !== Math.floor((aabb.x + aabb.hw) / this.partitionWidth)) ||
                            (Math.floor(intr.min.val.y / this.partitionHeight) !== Math.floor((aabb.y - aabb.hh) / this.partitionHeight)) ||
                            (Math.floor(intr.max.val.y / this.partitionHeight) !== Math.floor((aabb.y + aabb.hh) / this.partitionHeight))
                        ){
                            crossedBoundary = true;
                        }
                    }
    
                    // copy the position (plus or minus) the aabb half-dimensions
                    // into the min/max intervals
                    intr.min.val.clone( aabb ).sub( aabb.hw, aabb.hh );
                    intr.max.val.clone( aabb ).add( aabb.hw, aabb.hh );
    
                    if ( crossedBoundary ){
                        sectionsQuery = this.findIntersectingSections( intr.min.val, intr.max.val, true );
    
                        for ( sc = 0, sl = sections.length; sc < sl; sc++ ){
                            if ( sectionsQuery.indexOf( sections[ sc ] ) === -1 ){
                                // we moved out of this section
                                this.removeFromIntervalList( tr, sections[ sc ] );
                                sections.splice(sc, 1);
                                sc--;
                                sl--;
                            }
                        }
    
                        for ( sc = 0, sl = sectionsQuery.length; sc < sl; sc++ ){
                            if ( sections.indexOf( sectionsQuery[ sc ] ) === -1 ){
                                // we moved into a new section
                                this.addToIntervalList( tr, sectionsQuery[ sc ] );
                                sections.push(sectionsQuery[ sc ]);
                            }
                        }
                    }
                }
            },
    
            /** internal
             * SweepPruneBehavior#trackBody( data )
             * - data (Object): Event data
             *
             * Event callback to add body to list of those tracked by sweep and prune
             **/
            trackBody: function( data ){
    
                var body = data.body
                    ,aabb = data.body.aabb()
                    ,tracker = {
    
                        id: getUniqueId(),
                        body: body
                    }
                    ,intr = {
    
                        min: {
                            type: false, //min
                            val: new Physics.vector(),
                            tracker: tracker
                        },
    
                        max: {
                            type: true, //max
                            val: new Physics.vector(),
                            tracker: tracker
                        }
                    }
                    ,lists
                    ;
    
                lists = this.findIntersectingSections( intr.min.val, intr.max.val, true );
    
                tracker.interval = intr;
                tracker.sections = lists.slice();
                this.tracked.push( tracker );
    
                for ( var i = 0; i < lists.length; i++ ){
                    this.addToIntervalList( tracker, lists[ i ] );
                }
            },
    
            /** internal
             * SweepPruneBehavior#untrackBody( data )
             * - data (Object): Event data
             *
             * Event callback to remove body from list of those tracked
             **/
            untrackBody: function( data ){
    
                var body = data.body
                    ,list
                    ,minmax
                    ,trackedList = this.tracked
                    ,tracker
                    ,count
                    ,sections
                    ;
    
                for ( var i = 0, l = trackedList.length; i < l; ++i ){
    
                    tracker = trackedList[ i ];
    
                    if ( tracker.body === body ){
    
                        // remove the tracker at this index
                        trackedList.splice(i, 1);
    
                        sections = tracker.sections;
    
                        for ( var listNum = 0; listNum < sections.length; listNum++ ){
    
                            this.removeFromIntervalList( tracker, sections[listNum] );
                        }
    
                        break;
                    }
                }
            },
    
            addToIntervalList: function ( tracker, list ) {
                var intr = tracker.interval;
                for ( var xyz = 0; xyz < maxDof; ++xyz ){
                    list[ xyz ].push( intr.min, intr.max );
                }
            },
    
            removeFromIntervalList: function ( tracker, list ) {
    
                var innerlist, count, minmax;
    
                for ( var xyz = 0; xyz < maxDof; ++xyz ){
    
                    count = 0;
                    innerlist = list[ xyz ];
    
                    for ( var j = 0, m = innerlist.length; j < m; ++j ){
    
                        minmax = innerlist[ j ];
    
                        if ( minmax === tracker.interval.min || minmax === tracker.interval.max ){
    
                            // remove interval from list
                            innerlist.splice(j, 1);
                            j--;
                            m--;
    
                            if (count > 0){
                                break;
                            }
    
                            count++;
                        }
                    }
                }
            },
    
            /** internal
             * SweepPruneBehavior#sweep( data )
             * - data (Object): Event data
             *
             * Event callback to sweep and publish event if any candidate collisions are found
             **/
            sweep: function( data ){
    
                var self = this
                    ,candidates
                    ;
    
                candidates = self.broadPhase();
    
                if ( candidates.length ){
    
                    this._world.emit( this.options.channel, {
                        candidates: candidates
                    });
                }
            }
        };
    });
    
    // end module: behaviors/sweep-prune.js
    return Physics;
}));// UMD