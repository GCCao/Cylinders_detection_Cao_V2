function Median_PointDistances = Compute_Median_PointDistances(this, K)
            
            this.pcshs;
            
            % Reset K if there are not enough points
            K = min(double(K), this.Count);
            
            if this.Count <= 2
                neighboringSize = NaN;
                return;
            end
            
            this.buildKdtree();
            
            if this.isOrganized
                loc = reshape(this.Location, [], 3);
            else
                loc = this.Location;
            end
            
            % Use exact search in Kdtree
            searchOpts.eps = 0;
            % Setting the small grainsize (default = 2000) leads to
            % relatively high proportion of overheads. To overcome the
            % overheads caused by threads, need to increase the grainsize
            % with known parameter. TBB uses the auto_partitioner(default)
            % performs automatic chunk size as "grainsize/2 <= chunksize".
            % Choosing the optimum grainsize as "this.Count * 1.5" is
            % giving the significant performance. 
            searchOpts.grainSize = this.Count * 1.5;
            
            % Find K nearest neighbors for each point
            [indices, ~, valid] = this.Kdtree.knnSearch(loc, K, searchOpts);
            
            % Find normal vectors for each point
            normals = visionPCANormal(loc, indices, valid);
            
            if this.isOrganized
                normals = reshape(normals, size(this.Location));
            end
        end